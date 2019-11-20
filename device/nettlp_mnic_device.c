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
#include "nettlp_mnic_device.h"

#define Q_VECTORS	16
#define TX_QUEUES   	8
#define RX_QUEUES   	8

#define BASE_SUM	16
#define DESC_ENTRY_SIZE	256	

#define BAR4_TX_DESC_OFFSET 	64
#define BAR4_RX_DESC_OFFSET	128

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define is_mwr_addr_tx_desc_ptr(bar4,a)		\
	(a - bar4 <= BAR4_TX_DESC_OFFSET)

#define is_mwr_addr_rx_desc_ptr(bar4,a)		\
	(a - bar4 <= BAR4_TX_DESC_OFFSET)

struct tx_desc_ctl{
	uint32_t tx_head_idx;
	uintptr_t tx_desc_head;
};

struct rx_desc_ctl{
	uint32_t rx_head_idx;
	uintptr_t rx_desc_head;
};

struct descriptor{
	uint64_t addr;
	uint64_t length;
} __attribute__((packed));

struct nettlp_mnic{
	//tap fd 
	int tap_fd;
	uintptr_t bar4_start;
	int tx_queue_id;
	uintptr_t *tx_desc_base;
	int rx_queue_id;
	uintptr_t *rx_desc_base;

	struct nettlp *rx_nt; 
	struct nettlp_msix *tx_irq,*rx_irq;

	struct descriptor *rx_desc;
	struct tx_desc_ctl *tx_desc_ctl;
	struct rx_desc_ctl *rx_desc_ctl;

	unsigned char *tx_buf;
	unsigned char *rx_buf;
	
	int rx_state;
	unsigned int writev_cnt;
	uint8_t q_offset;
#define RX_STATE_INIT	0
#define RX_STATE_READY  1
#define RX_STATE_BUSY   2
#define RX_STATE_DONE	3
	uintptr_t *rx_desc_addr;
	
};

static inline unsigned int get_bar4_offset(uintptr_t start,uintptr_t received)
{
	unsigned int offset;
	
	offset = (received - start - BASE_SUM)/8;
	
	return offset;
}

int tap_alloc(char *dev)
{
        struct ifreq ifr;

        memset(&ifr,0,sizeof(ifr));
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

        if(ioctl(fd,SIOCSIFFLAGS,(void *)&ifr) < 0){
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

//rx_desc
void mnic_tx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int i=0;
	int ret;
	uintptr_t addr,tail;
	struct descriptor desc;
	struct iovec vector[idx];
	unsigned char *buf = mnic->tx_buf;
	struct tx_desc_ctl *txd_ctl = mnic->tx_desc_ctl + offset;
	struct nettlp_msix *tx_irq = mnic->tx_irq + offset;
	
	addr = *(mnic->tx_desc_base + offset);
	tail = addr + (sizeof(struct descriptor)*idx);
	
	while(txd_ctl->tx_head_idx != idx){
		ret = dma_read(nt,txd_ctl->tx_desc_head,&desc,sizeof(desc));
		if(ret < sizeof(struct descriptor)){
			if(ret < sizeof(struct descriptor)){
				fprintf(stderr,"failed to read tx desc from %#lx\n",txd_ctl->tx_desc_head);
				goto tx_end;
			}
		}
		
		ret = dma_read(nt,desc.addr,buf,desc.length);
		if(ret < desc.length){
			fprintf(stderr,"failed to read tx pkt from %#lx, %lu-byte",desc.addr,desc.length);
				goto tx_end;
		}
		
		vector[i].iov_base = buf;
		//pagesize
		vector[i].iov_len = 4096;

		txd_ctl->tx_head_idx++;
		txd_ctl->tx_desc_head += sizeof(struct descriptor);
		buf++;
		i++;

		if(txd_ctl->tx_head_idx >= DESC_ENTRY_SIZE){
			txd_ctl->tx_head_idx = 0;
			txd_ctl->tx_desc_head = addr;
		}
	}

	mnic->writev_cnt = i;
	mnic->q_offset = offset;
	ret = writev(mnic->tap_fd,vector,i);
	if(ret < 0){
		fprintf(stderr,"failed to tx pkt to tap \n");
		perror("write");
		goto tx_end;
	}

tx_end:
	ret = dma_write(nt,tx_irq->addr,&tx_irq->data,sizeof(tx_irq->data));
	if(ret < 0){
		fprintf(stderr,"failed to send tx interrupt\n");
		perror("dma_write");
	}
	
	txd_ctl->tx_desc_head = tail;
	printf("TX done");
}

//receive用のバッファの再利用
void mnic_rx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int ret;
	struct descriptor *desc = mnic->rx_desc;
	uintptr_t *rx_desc_base = mnic->rx_desc_base + offset;
	struct rx_desc_ctl *rxd_ctl = mnic->rx_desc_ctl + offset;
	//uintptr_t addr;
	
	if(*rx_desc_base == 0){
		fprintf(stderr,"rx_desc base is 0\n");
		return;
	}

	while(mnic->rx_state != RX_STATE_DONE && mnic->rx_state != RX_STATE_INIT){
		sched_yield();
	}

	//addr = rxd_ctl->rx_desc_head + (sizeof(struct descriptor)*idx);
	//addr = rxd_ctl->rx_desc_head;
	
	while(rxd_ctl->rx_head_idx != idx){
		ret = dma_read(nt,rxd_ctl->rx_desc_head,desc,sizeof(struct descriptor));
		if(ret < sizeof(struct descriptor)){
			fprintf(stderr,"failed to read rx desc from %#lx\n",rxd_ctl->rx_desc_head);
			return;
		}
		 	
		rxd_ctl->rx_desc_head += sizeof(struct descriptor);
		rxd_ctl->rx_head_idx++;
		desc++;

		if(rxd_ctl->rx_head_idx > DESC_ENTRY_SIZE){	
			rxd_ctl->rx_head_idx = 0;
			rxd_ctl->rx_desc_head = *rx_desc_base;
		}
	}

	mnic->rx_nt = nt;
	mnic->rx_state = RX_STATE_READY;
}

int nettlp_mnic_mwr(struct nettlp *nt,struct tlp_mr_hdr *mh,void *m,size_t count,void *arg)
{
	unsigned int offset;
	uint32_t idx;
	struct nettlp_mnic *mnic = arg;
	uintptr_t dma_addr;

	dma_addr = tlp_mr_addr(mh);
	info("dma_addr is %#lx",dma_addr);
	
	if(is_mwr_addr_tx_desc_ptr(mnic->bar4_start,dma_addr)){
		uintptr_t *txd_base = mnic->tx_desc_base + mnic->tx_queue_id;
		struct tx_desc_ctl *txd_ctl = mnic->tx_desc_ctl + mnic->tx_queue_id;
		memcpy(txd_base,m,8);
		memcpy(&txd_ctl->tx_desc_head,m,8);
		info("Queue %d: TX desc base is %lx",mnic->tx_queue_id,*txd_base);
		mnic->tx_queue_id++;
	}
	else if(is_mwr_addr_rx_desc_ptr(mnic->bar4_start,dma_addr)){
		uintptr_t *rxd_base = mnic->rx_desc_base + mnic->rx_queue_id;
		struct rx_desc_ctl *rxd_ctl = mnic->rx_desc_ctl + mnic->rx_queue_id;
		memcpy(rxd_base,m,8);
		memcpy(&rxd_ctl->rx_desc_head,m,8);
		info("Queue %d: RX desc base is %lx",mnic->rx_queue_id,*rxd_base);
		mnic->rx_queue_id++;
	}
	else{
		offset = get_bar4_offset(mnic->bar4_start,dma_addr);
		memcpy(&idx,m,sizeof(idx));
		if(offset < 8){
			mnic_tx(idx,nt,mnic,offset);
			return 0;
		}else if(offset >= 8){
			mnic_rx(idx,nt,mnic,offset/8);
			return 0;
		}
	}

	return 0;
}

/*actual rx part*/
void *nettlp_mnic_tap_read_thread(void *arg)
{
	int ret,pktlen,i,v;
	int q_offset;
	struct nettlp_mnic *mnic = arg;
	unsigned char *buf = mnic->rx_buf;
	uintptr_t *rxd_addr;
	struct nettlp_msix *rx_irq;
	//struct descriptor *desc = mnic->rx_desc;
	struct iovec vector[DESC_ENTRY_SIZE];
	struct pollfd x[1] = {{.fd = mnic->tap_fd, .events = POLLIN}};

	while(1){
		if(caught_signal){
			break;
		}
		
		ret = poll(x,1,500);
		if(ret < 0 || ret == 0 || !(x[0].revents & POLLIN)){
			continue;
		}

		v = mnic->writev_cnt;	
		q_offset = mnic->q_offset;	
		rxd_addr = mnic->rx_desc_addr + q_offset;
		rx_irq = mnic->rx_irq + q_offset;

		for(i=0;i<v;i++){
			vector[i].iov_base = buf;
			vector[i].iov_len = 4096;
			buf++;
		}

		pktlen = readv(mnic->tap_fd,vector,i);
		if(pktlen<0){
			perror("read");
			continue;
		}

		if(mnic->rx_state != RX_STATE_READY){
			continue;
		}
	
		mnic->rx_state = RX_STATE_BUSY;

		//dma_write packets to host
		for(v=0;v<i;v++){
			ret = dma_write(mnic->rx_nt,mnic->rx_desc->addr,vector[v].iov_base,4096);
			if(ret < 0){
				debug("failed to dma_write pkt to %lx",mnic->rx_desc->addr);
				continue;
			}
			mnic->rx_desc++;
		}
		
		//mnic->rx_desc.length = pktlen;
		//write back 問題
		ret = dma_write(mnic->rx_nt,*rxd_addr,mnic->rx_desc,sizeof(struct descriptor));
		if(ret < sizeof(struct descriptor)){
			fprintf(stderr,"failed to write rx desc to %#lx\n",*rxd_addr);
			continue;
		}

		ret = dma_write(mnic->rx_nt,rx_irq->addr,&rx_irq->data,sizeof(rx_irq->data));
		if(ret < 0){
			fprintf(stderr,"failed to generate Rx Interrupt\n");
			perror("dma_write for rx interrupt");
		}

		info("RX done. DMA write to %lx %d byte",*rxd_addr,pktlen);
		mnic->rx_state = RX_STATE_DONE;
	}

	return NULL;
}

void mnic_alloc(struct nettlp_mnic *mnic)
{
	mnic->tx_desc_base = calloc(TX_QUEUES,sizeof(uintptr_t));
	mnic->rx_desc_base = calloc(RX_QUEUES,sizeof(uintptr_t));
	mnic->rx_desc_addr = calloc(RX_QUEUES,sizeof(uintptr_t));

	mnic->tx_irq = calloc(TX_QUEUES,sizeof(struct nettlp_msix));
	mnic->rx_irq = calloc(RX_QUEUES,sizeof(struct nettlp_msix));

	mnic->tx_buf = calloc(DESC_ENTRY_SIZE,4096);
	mnic->rx_buf = calloc(DESC_ENTRY_SIZE,4096);

	mnic->rx_desc = calloc(DESC_ENTRY_SIZE,sizeof(struct descriptor));
	mnic->tx_desc_ctl = calloc(TX_QUEUES,sizeof(struct tx_desc_ctl));
	mnic->rx_desc_ctl = calloc(RX_QUEUES,sizeof(struct rx_desc_ctl));
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
        int opt,ret,tap_fd,i,n;
        char *ifname = "tap0";
	struct nettlp nt,nts[16],*nts_ptr[16];
	struct nettlp_cb cb;
	struct in_addr host;
	struct nettlp_mnic mnic;	
	struct nettlp_msix msix[16];
	pthread_t rx_tid; //tap_read_thread

        while((opt = getopt(argc,argv,"t:r:l:R:")) != -1){
                switch(opt){
                case 't':
                        ifname = optarg;
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
        if(tap_fd < 0){
                perror("failed to allocate tap");
                return -1;
        }
        
        if(tap_up(ifname) < 0){
                perror("failed to up tap");
                return -1;
        }
        
	mnic_alloc(&mnic);

	for(n=0;n<16;n++){
		nts[n] = nt;
 		nts[n].tag = n;
		nts_ptr[n] = &nts[n];

		ret = nettlp_init(nts_ptr[n]);
		if(ret < 0){
			debug("failed to init nettlp on tag %x\n",n);
			return ret;
		}
	}

	mnic.tap_fd = tap_fd;
	mnic.bar4_start = nettlp_msg_get_bar4_start(host);	
	if(mnic.bar4_start == 0){
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
		*mnic.tx_irq = msix[i];
		*mnic.rx_irq = msix[i+8];
		mnic.tx_irq++;
		mnic.rx_irq++;
	}

	info("Device is %04x",nt.requester);
	info("BAR4 start adress is %#lx",mnic.bar4_start);	       
	info("TX IRQ address is %#lx,data is 0x%08x",mnic.tx_irq->addr,mnic.tx_irq->data);
	info("RX IRQ address is %#lx,data is 0x%08x",mnic.rx_irq->addr,mnic.rx_irq->data);

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