#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <signal.h>
#include <poll.h>
#include <pthread.h>

#include <libtlp.h>
#include <nettlp_mnic.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define IDX_OFFSET	8
#define BASE_SUM	16
#define DESC_ENTRY_SIZE	256	

static int rxd_idx;

struct desc_ctl{
	uint32_t tx_head_idx;
	uint32_t rx_head_idx;
	uint64_t tx_desc_head;
	uint64_t rx_desc_head;
};

struct nettlp_mnic{
	//tap fd 
	int tap_fd;
	uintptr_t bar4_start;
	int tx_queue_id;
	uintptr_t tx_desc_base[10]:
	int rx_queue_id;
	uintptr_t rx_desc_base[10];

	struct nettlp *rx_nt; 
	struct nettlp_msix tx_irq[8],rx_irq[8];
	struct device_register dev_reg;

	char *tx_buf[];
	char *rx_buf[];
	
	int rx_state;
	int writev_cnt;
#define RX_STATE_INIT	0
#define RX_STATE_READY  1
#define RX_STATE_BUSY   2
#define RX_STATE_DONE	3

	uintptr_t rx_desc_addr[DESC_SIZE];
	struct descriptor rx_desc;
};

static inline unsigned int get_bar4_offset(uintptr_t start,uintptr_t received)
{
	unsigned int offset;
	
	offset = (received-start-BASE_SUM)/IDX_OFFSET;
	
	return offset;
}

int tap_alloc(char *dev)
{
        struct ifreq ifr;

        memset(&ifr,0,sizof(ifr));
        ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        strncpy(ifr.ifr_name,dev,IFNAMSIZ);

        int fd = open("/dev/net/tap",O_RDWR);
        if(fd < 0){
                perror("open");
                return -1;
        }

        if(ioctl(fd,TUNSETIFF,(void *)&ifr)<0){
                perror("ioctl");
                close(fd);
                return -1;
        }

        return fd;
}

int tap_up(char *dev)
{
        struct ifreq ifr;

        int fd = socket(AF_INET,SOCK_DGRAM,0);
        if(fd < 0){
                perror("socket");
                return -1;
        }

        memset(&ifr,0,sizeof(ifr));
        ifr.ifr_flags = IFF_UP;
        strncpy(ifr.ifr_name,dev,IFNAMSIZ);

        if(ioctl(fd,SIOCSIFFAGS,(void *)&ifr) < 0){
                perror("ioctl");
                return -1;
        }
        close(fd);
        return 0;
}

static int caught_signal = 0;

void signal_handler(int signal)
{
	caught_signal = 1;
	nettlp_stop_cb();
}

//headの取得
//一周回ったあとの0への戻り方
void mnic_tx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int i=0;
	int ret;
	uintptr_t addr;
	struct descriptor desc;
	struct iovec vector[idx];
	//char *buf[] = malloc(4096)*WRITEV_SIZE;

	addr = mnic->tx_desc_base[offset] + (sizeof(struct descriptor)*idx);

	while(/*mnic->tx_head != addr*/mnic->tx_head_idx != idx){
		ret = dma_read(nt,mnic->head,&desc,sizeof(desc));
		if(ret < sizeof(desc)){
			if(ret < sizeof(desc)){
				fptintf(stderr,"failed to read tx desc from %#lx\n",addr);
				goto tx_end;
			}
		}
		
		ret = dma_read(nt,desc.addr,mnic->tx_buf[i],desc.length);
		if(ret < desc.length){
			fprintf(stderr,"failed to read tx pkt from %#lx, %lu-byte",desc.addr,desc.length);
				goto tx_end;
		}
		
		vector[i].iov_base = mnic->tx_buf[i];
		vector[i].iov_len = sizeof(4096);

		mnic->tx_head_idx++;
		mnic->tx_head += (sizeof(struct descriptor));
		i++;

		if(mnic->tx_head_idx >= DESC_ENTRY_SIZE){
			mnic->tx_head_idx = 0;
			mnic->tx_head = mnic->tx_desc_base[offset];
		}
	}

	mnic->writev_cnt = i;
	ret = writev(mnic->tap_fd,vector,i);
	if(ret < 0){
		fprintf(stderr,"failed to tx pkt to tap \n");
		perror("write");
		goto tx_end;
	}

tx_end:
	ret = dma_write(nt,mnic->tx_irq.addr,&mnic->tx_irq[offset].data,sizeof(mnic->tx_irq[offset].data));
	if(ret < 0){
		fprintf(stderr,"failed to send tx interrupt\n");
		perror("dma_write");
	}
	
	mnic->head = addr;
	printf("TX done");
}

void mnic_rx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int i=0;
	int ret;
	uintptr_t addr;
	
	if(mnic->rx_desc_base[offset] == 0){
		fprintf(stderr,"rx_desc base is 0\n");
		return;
	}

	while(mnic->rx_state != RX_STATE_DONE && mnic->rx_state != RX_STATE_INIT){
		sched_yield();
	}

	addr = mnic->rx_desc_base[offset] + (sizeof(struct descriptor)*idx);
	
	while(/*mnic->rx_head != addr*/mnic->rx_head_idx != idx){
		//mnic->rx_desc_addr[rxd_offset] = mnic->rx_head;

		ret = dma_read(nt,mnic->rx_desc_addr[i],&mnic->rx_desc,sizeof(mnic->rx_desc));
		if(ret < sizeof(mnic->rx_desc)){
			fprintf(stderr,"failed to read rx desc from %#lx\n",mnic->rxdesc_addr[i]);
			return;
		}
		 	
		mnic->rx_head += (sizeof(struct descriptor));
		i++;
		mnic->rx_head_idx++;
		if(mnic->rx_head_idx > DESC_ENTRY_SIZE){
			mnic->rx_head_idx = 0;
			mnic->rx_head = mnic->rx_desc_base[offset];
		}
	}

	mnic->rx_nt = nt;
	mnic->rx_state = RX_STATE_READY;
}

int nettlp_mnic_mwr(struct nettlp *nt,struct tlp_mr_hdr *mr,void *m,size_t count,void *arg)
{
	int i=0;
	unsigned int offset;
	uint64_t tail,ret;
	uint32_t idx;
	struct nettlp_mnic *mnic = arg;
	uintptr_t dma_addr,addr;

	dma_addr = tlp_mr_addr(mh);
	info("dma addr is %lx\n",mh);
	
	if(is_mwr_addr_tx_desc_ptr(mnic->bar4_start,dma_addr)){
		memcpy(&mnic->tx_desc_base[mnic->tx_queue_id],m,8);
		memcpy(&mnic->dev_reg.tx_desc_head[mnic->tx_queue_id],m,8);
		info("Queue %d: TX desc base is %lx",mnic->tx_queue_id,mnic->tx_desc_base);
		mnic->tx_queue_id++;
	}
	else if(is_mwr_addr_rx_desc_ptr(mnic->bar4_start,dma_addr)){
		memcpy(&mnic->rx_desc_base[mnic->rx_queue_id],m,8);
		memcpy(&mnic->dev_reg.rx_desc_head[mnic->rx_queue_id],m,8);
		info("Queue %d: RX desc base is %lx",mnic->rx_queue_id,mnic->rx_desc_base);
		mnic->rx_queue_id++;
	}
	else{
		offset = get_bar4_offset(mnic->bar4_start,dma_addr);
		memset(&idx,m,sizeof(idx));
		if(offset < 8){
			mnic_tx(idx,nt,mnic,offset);
			return 0;
		}else if(offset >= 8){
			mnic_rx;
			return 0;
		}
	}
		/*
		   1.get tx descriptor ring
		   2.see the pkd addr
		   3.dma the pkt of 2
		   4.dma_write to the adapter
		*/
	/*else if(is_mwr_addr_tx_tail_ptr(mnic->bar4_start,dma_addr)){
		if(unlikely(mnic->tx_desc_base == 0)){
			debug("tx desc base is NULL");
			goto tx_end;
		}
			memcpy(&tail,m,sizeof(tail));
	}
	else if(is_mwr_addr_rx_tail_ptr(mnic->bar4_start,dma_addr)){
		if(unlikely(mnic->rx_desc_base == 0)){
			debug("rx desc base is NULL");
			goto rx_end;
		}

	}*/
	return 0;
}

/*actual rx part*/
void *nettlp_mnic_tap_read_thread(void *arg)
{
	int ret,len,i,v;
	//char *buf[] = malloc(4096)*WRITEV_SIZE;
	struct nettlp_mnic *mnic = arg;
	struct iovec vector[DESC_ENTRY_SIZ];
	struct poll_fd x[1] = {{.fd = mnic->tap_fd,.events = POLLIN}};

	while(1){
		if(caught_signal){
			break;
		}
		
		ret = poll(x,1,500);
		if(ret < 0 || ret == 0 || !(x[0].revents & POLLIN)){
			continue;
		}

		v = mnic->writev_cnt;	
		for(i=0;i<v;i++){
			vector[i]->iov_base = mnic->rx_buf[i];
			vector[i]->iov_len = 4096;
		}

		len = readv(mnic->tap_fd,vector,i);
		if(len<0){
			perror("read");
			continue;
		}

		if(mnic->rx_state != RX_STATE_READY){
			continue;
		}
	
		mnic->rx_state = RX_STATE_BUSY;

		for(v=0;v<i;v++){
			ret = dma_write(mnic->rx_nt,mnic->rx_desc.addr,vector[i].iov_base,vector[i].iov_len);
			if(ret < 0){
				debug("failed to dma_write pkt to %lx",mnic->rx_desc.addr);
				continue;
			}
		}
		
		mnic->rx_desc.length = pktlen;
		ret = dma_write(mnic->rx_nt,mnic->rx_desc_addr,&mnic->rx_desc,sizeof(mnic->rx_desc));
		if(ret < sizeof(mnic->rx_desc)){
			fprintf(stderr,"failed to write rx desc to %#lx\n",mnic->rx_desc_addr);
			continue;
		}

		ret = dma_write(mnic->rx_nt,mnic->rx_irq.addr,&mnic->rx_irq.data,sizeof(mnic->rx_irq.data));
		if(ret < 0){
			fprintf(stderr,"failed to generate Rx Interrupt\n");
			perror("dma_write for rx interrupt");
		}

		info("RX done. DMA write to %lx %d byte",mnic->rx_desc,len);
		mnic->rx_state = RX_STATE_DONE;
	}

	return NULL;
}

void usage()
{
	printf("usage\n"
	       "    -r remote addr\n"
	       "    -l local addr\n"
	       "    -R remote host addr (not TLP NIC)\n"
	       "\n"
	       "    -t tunif name (default tap0)\n"
		);	
}

	/*
		tx
		1.host updates the tx queue tail pointer
		2.device dmas descriptor
		3.device dmas packet content
		4.device writes back tx descriptor(optional)
		---omitted---
		5.device generate interrupt??
		6.host reads tx queue head pointer
		---omitted---
		device can fetch up to 40 tx descriptor
		device may prefetch tx descriptors if its internal Q
		becomes close to empty.
		all descriptor are 128 bit

		rx
		1.host updates rx queue tail pointer -> free buf
		2.device dmas descriptor from host
		3.device dmas packet to host
		4.device writes back rx descriptor
		---ommited---
		5.devicce generates interrupt
		6.host read rx queue head pointer
		---ommited---
		niantic does not prefetch freelist descriptors(on demand)
		niantic does not seem to be doing any batching of rx descriptor write-back.
	*/

int main(int argc,char **argv)
{
        int opt,ret,tap_fd,i;
        char *ifname = "tap8";
	struct nettlp nt,nts[16],*nts_ptr[16];
	struct nettlp nttlp_cb;
	struct in_addr host;
	struct nettlp_mnic mnic;	
	pthread_t rx_tid; //tap_read_thread

        while((opt = getopt(argc,argv,"t:r:l:R:")) != -1){
                switch(opt){
                case 't':
                        ifname = optarg:
                        break;
		case 'r':
			ret = inet_pton(AF_INET,optarg,&nt.remote_addr);
			if(ret < 1){
				perror("inet_pton");
				return -1;
			}
			break;
		case 'l':
			ret = inet_pton(AF_INET,optarg,&nt.local_addr);
			if(ret < 1){
				perror("inet_pton");
				return -1;
			}
			break;
		case 'R':
			ret = inet_pton(AF_INET,optarg,&host);
			if(ret < 1){
				perror("inet_pton");
				return -1;
			}
			break;
		default:
			usage();
			return -1;
                }
        }

	memset(&nt,0,sizeof(nt));
	memset(&mnic,0,sizeof(mnic));

        tap_fd = tap_alloc(ifname);
        if(fd < 0){
                perror("failed to allocate tap");
                return -1;
        }
        
        if(tap_up(ifname) < 0){
                perror("failed to up tap");
                return -1;
        }
        
	for(n=0;n<16;n++){
		nts[n] = nt;
 		nts[n].tag = n;
		nts_ptr[n] = &nts[n];

		ret = nettlp_init(nts_ptr[n]);
		if(ret < 0){
			debug("failed to init nettlp on tag %x\n",n):
			return ret;
		}
	}

	mnic.fd = tap_fd;
	mnic.bar4_start = nettlp_msg_get_bar4_start(host);	
	if(snic.bar4_start == 0){
		debug("failed to get BAR4 addr from %s\n",inet_ntoa(host));
		info("nettlp_msg_get_bar4_start");
		return -1;
	}

	ret = nettlp_msg_get_msix_table(host,msix,16);
	if(ret < 0){
		debug("faled to get msix table from %s\n",inet_ntoa(host));
		info("nettlp_msg_get_msix_table");
	}	

	for(i=0;i<8;i++){
		mnic.tx_irq[i] = msix[i];
		mnic.rx_irq[i+8] = msix[i+8];
	}

	mnic.tx_buf = malloc(4096)*DESC_ENTRY_SIZ;
	mnic.rx_buf = malloc(4096)*DESC_ENTRY_SIZ;

	mnic.dev_reg.tx_desc_head = 0;
	mnic.dev_reg.rx_desc_head = 0;
	mnic.tx_queue_id = 0;
	mnic.rx_queue_id = 0;
	rxd_idx = 0;

	info("Device is %04x",nt.requester);
	info("BAR4 start adress is %#lx",mnic.bar4_start);	       
	info("TX IRQ address is %#lx,data is 0x%08x",mnic.tx_irq.addr,mnic.tx_irq.data);
	info("RX IRQ address is %#lx,data is 0x%08x",mnic.rx_irq.addr,mnic.rx_irq.data);

	if(signal(SIGINT,signal_handler)==SIG_ERR){
		debug("failed to set signal");
		return -1;
	}

	info("create tap read thread\n");
	pthread_create(&rx_tid,NULL,nettlp_mnic_tap_read_thread,&mnic);
	
	info("start nettlp callback");
	memset(&cb,0,sizeof(cb));
	cb.mwr = nettlp_mnic_mwr;
	nettlp_run_cb(nts_ptr,16,&cb,&mnic);

	info("nettlp callback done");

	pthread_join(rx_tid,NULL);

        return 0;
}
