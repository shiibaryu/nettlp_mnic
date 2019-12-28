#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <signal.h>
#include <poll.h>
#include <pthread.h>

#include <libtlp.h>
#include "nettlp_mnic_device.h"

#define NUM_OF_CORE	8	

#define Q_VECTORS	16
#define TX_QUEUES   	8
#define RX_QUEUES   	8

#define DESC_ENTRY_SIZE 256	

#define BAR4_TX_DESC_OFFSET 	56	
#define BAR4_RX_DESC_OFFSET	120
#define BASE_SUM	128

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define is_mwr_addr_tx_desc_ptr(bar4,a)		\
	(a - bar4 <= BAR4_TX_DESC_OFFSET)

#define is_mwr_addr_rx_desc_ptr(bar4,a)		\
	(a - bar4 <= BAR4_RX_DESC_OFFSET)


struct tx_desc_ctl{
	uint32_t tx_head_idx;
	uintptr_t tx_desc_head;
	uint32_t tx_tail_idx;
	uintptr_t tx_desc_tail;
};

struct rx_desc_ctl{
	uint32_t rx_head_idx;
	uint32_t rx_tail_idx;
	uintptr_t rx_desc_head;
	uintptr_t rx_desc_tail;
};

struct tap_rx_ctl{
	int tap_fd;
	int *rx_state;
	pthread_t tid;
	struct nettlp *rx_nt;
	struct descriptor *rx_desc;
	struct nettlp_msix *rx_irq;
	struct rx_desc_ctl *rxd_ctl;
};
struct descriptor{
	uint64_t addr;
	uint64_t length;
} __attribute__((packed));

struct nettlp_mnic{
	int tap_fd;
	uintptr_t bar4_start;
	int tx_queue_id;
	uintptr_t *tx_desc_base;
	int rx_queue_id;
	uintptr_t *rx_desc_base;

	struct nettlp rx_nt; 
	struct nettlp_msix *tx_irq,*rx_irq;

	struct descriptor *tx_desc[TX_QUEUES];
	struct descriptor *rx_desc[RX_QUEUES];
	struct tx_desc_ctl *tx_desc_ctl;
	struct rx_desc_ctl *rx_desc_ctl;

	unsigned char *tx_buf;
	unsigned char *rx_buf;
	
	uint16_t write_cnt;
	uint8_t q_offset;
	int rx_state[NUM_OF_CORE];
#define RX_STATE_INIT	0
#define RX_STATE_READY  1
#define RX_STATE_BUSY   2
#define RX_STATE_DONE	3
	uintptr_t *rx_desc_addr;
#define _GNU_SOURCE
};

int tap_alloc(char *dev)
{
        struct ifreq ifr;

        memset(&ifr,0,sizeof(ifr));
        ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        strncpy(ifr.ifr_name,dev,IFNAMSIZ);

        int fd = open("/dev/net/tun",O_RDWR);
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
	size_t len;
        struct ifreq ifr;
	socklen_t i = sizeof(len);

        int fd = socket(AF_INET,SOCK_DGRAM,0);
        if(fd < 0){
                perror("socket");
                return -1;
        }

	/*set an option for a cpu affinity of socket*/
	if(setsockopt(fd,SOL_SOCKET,SO_INCOMING_CPU,&len,i)<0){
		perror("setsockopt");
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

void mnic_tx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int ret;
	struct descriptor *tx_desc = mnic->tx_desc[offset];
	unsigned char *buf = mnic->tx_buf + offset;
	struct tx_desc_ctl *txd_ctl = mnic->tx_desc_ctl + offset;
	struct nettlp_msix *tx_irq = mnic->tx_irq + offset;
	uintptr_t *tx_desc_base = mnic->tx_desc_base + offset;
	
	while(txd_ctl->tx_tail_idx != idx){
		ret = dma_read(nt,txd_ctl->tx_desc_tail,tx_desc,sizeof(struct descriptor));
		if(ret < sizeof(struct descriptor)){
			if(ret < sizeof(struct descriptor)){
				debug("failed to read tx desc from %#lx\n",txd_ctl->tx_desc_tail);
				goto tx_end;
			}
		}
	 	info("dma_write addr at offset %d is %#lx",offset,tx_desc->addr);
		
	 	info("dma_read");
		ret = dma_read(nt,tx_desc->addr,buf,tx_desc->length);
		if(ret < tx_desc->length){
			debug("failed to read tx pkt from %#lx, %lu-byte",tx_desc->addr,tx_desc->length);
				goto tx_end;
		}

		txd_ctl->tx_tail_idx++;
		txd_ctl->tx_desc_tail += sizeof(struct descriptor);
		buf++;
		tx_desc++;

		if(txd_ctl->tx_tail_idx >= DESC_ENTRY_SIZE - 1){
			txd_ctl->tx_tail_idx = 0;
			txd_ctl->tx_desc_tail = *tx_desc_base;
		}
	}

	buf = mnic->tx_buf + offset;
	tx_desc = mnic->tx_desc[offset];

	while(txd_ctl->tx_head_idx != txd_ctl->tx_tail_idx){
	 	info("write");
		ret = write(mnic->tap_fd,buf,tx_desc->length);
		if(ret < tx_desc->length){
			fprintf(stderr,"failed to read tx pkt from %lx,%lu-bytes\n",tx_desc->addr,tx_desc->length);
			perror("write");
			goto tx_end;
		}

		buf++;
		tx_desc++;
		txd_ctl->tx_head_idx++;

		if(txd_ctl->tx_head_idx > DESC_ENTRY_SIZE){
			txd_ctl->tx_head_idx = 0;
		}
	}

tx_end:
 	info("dma_write irq");
	ret = dma_write(nt,tx_irq->addr,&tx_irq->data,sizeof(tx_irq->data));
	if(ret < 0){
		fprintf(stderr,"failed to send tx interrupt\n");
		perror("dma_write");
	}
	
	printf("TX done");
}

void mnic_rx(uint32_t idx,struct nettlp *nt,struct nettlp_mnic *mnic,unsigned int offset)
{
	int ret;
	int cpu = sched_getcpu();
	struct descriptor *rx_desc = mnic->rx_desc[offset];
	uintptr_t *rx_desc_base = mnic->rx_desc_base + offset;
	struct rx_desc_ctl *rxd_ctl = mnic->rx_desc_ctl + offset;

	if(*rx_desc_base == 0){
		fprintf(stderr,"rx_desc base is 0\n");
		return;
	}

	while(mnic->rx_state[offset] != RX_STATE_DONE && mnic->rx_state[offset] != RX_STATE_INIT){
		sched_yield();
	}
	
	while(rxd_ctl->rx_tail_idx != idx){
		ret = dma_read(nt,rxd_ctl->rx_desc_tail,rx_desc,sizeof(struct descriptor));
		if(ret < sizeof(struct descriptor)){
			fprintf(stderr,"failed to read rx desc from %#lx\n",rxd_ctl->rx_desc_tail);
			return;
		}
		//info("dma_read addr at queue %d & cpu %d is %#lx",offset,cpu,rx_desc->addr);
		
		rx_desc++;
		rxd_ctl->rx_tail_idx++;
		rxd_ctl->rx_desc_tail += sizeof(struct descriptor);
	
		if(rxd_ctl->rx_tail_idx > DESC_ENTRY_SIZE - 1){	
			info("yes");
			rxd_ctl->rx_tail_idx = 0;
			rxd_ctl->rx_desc_tail = *rx_desc_base;
		}
	}
	
	//info("rx_head index is %d",rxd_ctl->rx_head_idx);
	//info("rx_tail index is %d",rxd_ctl->rx_tail_idx);

	mnic->rx_nt = *nt;
	mnic->rx_state[offset] = RX_STATE_READY;
}

static inline unsigned int get_bar4_offset(uintptr_t start,uintptr_t received)
{
	unsigned int offset;
	
	//info("start %#lx, received %#lx",start,received);

	offset = (received - start - BASE_SUM)/8;
	
	return offset;
}

int nettlp_mnic_mwr(struct nettlp *nt,struct tlp_mr_hdr *mh,void *m,size_t count,void *arg)
{
	unsigned int offset;
	uint32_t idx;
	struct nettlp_mnic *mnic = arg;
	uintptr_t dma_addr;

	dma_addr = tlp_mr_addr(mh);
	//info("dma_addr is %#lx",dma_addr);
	
	if(is_mwr_addr_tx_desc_ptr(mnic->bar4_start,dma_addr)){
		uintptr_t *txd_base = mnic->tx_desc_base + mnic->tx_queue_id;
		struct tx_desc_ctl *txd_ctl = mnic->tx_desc_ctl + mnic->tx_queue_id;
		memcpy(txd_base,m,8);
		txd_ctl->tx_desc_head = *txd_base;
		txd_ctl->tx_desc_tail = *txd_base;
		info("Queue %d: TX desc base is %#lx",mnic->tx_queue_id,*txd_base);
		//info("head %#lx, tail %#lx",txd_ctl->tx_desc_head,txd_ctl->tx_desc_tail);
		mnic->tx_queue_id++;
	}
	else if(is_mwr_addr_rx_desc_ptr(mnic->bar4_start,dma_addr)){
		uintptr_t *rxd_base = mnic->rx_desc_base + mnic->rx_queue_id;
		struct rx_desc_ctl *rxd_ctl = mnic->rx_desc_ctl + mnic->rx_queue_id;
		memcpy(rxd_base,m,8);
		rxd_ctl->rx_desc_head = *rxd_base;
		rxd_ctl->rx_desc_tail = *rxd_base;
		info("Queue %d: RX desc base is %lx",mnic->rx_queue_id,*rxd_base);
		//info("head %#lx, tail %#lx",rxd_ctl->rx_desc_head,rxd_ctl->rx_desc_tail);
		mnic->rx_queue_id++;
	}
	else{
		offset = get_bar4_offset(mnic->bar4_start,dma_addr);
		memcpy(&idx,m,sizeof(idx));
		//info("offset is %d",offset);
		if(offset < 8){
			info("go to tx");
			mnic_tx(idx,nt,mnic,offset);
			return 0;
		}else if(offset >= 8){
			//info("go to rx");
			mnic_rx(idx,nt,mnic,offset - 8);
			return 0;
		}
	}

	return 0;
}

/*actual rx part*/
void *nettlp_mnic_tap_read_thread(void *arg)
{
	int ret,pktlen;
	int cpu = sched_getcpu();
	struct tap_rx_ctl *tap_rx_ctl = arg;
	char buf[2048];
	uintptr_t rxd_addr;
	int tap_fd = tap_rx_ctl->tap_fd;
	int *rx_state = tap_rx_ctl->rx_state;
	struct descriptor *rx_desc = tap_rx_ctl->rx_desc;
	struct nettlp_msix *rx_irq = tap_rx_ctl->rx_irq;
	struct rx_desc_ctl *rxd_ctl = tap_rx_ctl->rxd_ctl;
	struct nettlp *rx_nt = tap_rx_ctl->rx_nt;
	struct pollfd x[1] = {{.fd = tap_rx_ctl->tap_fd, .events = POLLIN}};

	while(1){
		if(caught_signal){
			break;
		}
		
		ret = poll(x,1,500);

		if(ret < 0 || ret == 0 || !(x[0].revents & POLLIN)){
			continue;
		}
		/*
		if(rxd_ctl->rx_tail_idx == 0 && rxd_ctl->rx_head_idx == 0){
			debug("rx_desc is not prepared yet");
			continue;
		}*/
		
		pktlen = read(tap_fd,buf,sizeof(buf));
		if(pktlen < 0){
			perror("read");
			continue;
		}

		if(*rx_state != RX_STATE_READY){
			continue;
		}
		
		*rx_state = RX_STATE_BUSY;
		rxd_addr = rxd_ctl->rx_desc_head;
		
		//ret = dma_write(rx_nt,rxd_ctl->rx_desc_head,buf,pktlen);
		ret = dma_write(rx_nt,rx_desc->addr,buf,pktlen);
		if(ret < 0){
			debug("buf to rx_desc: failed to dma_write to %lx",rx_desc->addr);
			continue;
		}

		rxd_ctl->rx_desc_head += sizeof(struct descriptor);
		rxd_ctl->rx_head_idx++;
		
		//write back
		rx_desc->length = pktlen;
		ret = dma_write(rx_nt,rxd_addr,rx_desc,sizeof(struct descriptor));
		if(ret < 0){
			debug("rx_desc write_back: failed to dma_write to %#lx",rxd_addr);
			continue;
		}
		info("dma_write back done");

		//generate rx interrupt
		ret = dma_write(rx_nt,rx_irq->addr,&rx_irq->data,sizeof(rx_irq->data));
		if(ret < 0){
			fprintf(stderr,"failed to generate Rx Interrupt\n");
			perror("dma_write for rx interrupt");
		}
		
		info("RX done. DMA write at cpu %d to %lx %d byte, Data is %s",cpu,rxd_ctl->rx_desc_head,pktlen,buf);
		info("rx_irq addr is %#lx, data is %08x",rx_irq->addr,rx_irq->data);
		
		if(rxd_ctl->rx_head_idx < rxd_ctl->rx_tail_idx){
			*rx_state = RX_STATE_READY;
		}
		else{
			*rx_state = RX_STATE_DONE;
		}
	}
	
	pthread_join(tap_rx_ctl->tid,NULL);

	return NULL;
}

void mnic_alloc(struct nettlp_mnic *mnic)
{
	mnic->tx_desc_base = calloc(TX_QUEUES,sizeof(uintptr_t));
	mnic->rx_desc_base = calloc(RX_QUEUES,sizeof(uintptr_t));
	mnic->rx_desc_addr = calloc(RX_QUEUES,sizeof(uintptr_t));

	mnic->tx_irq = calloc(TX_QUEUES,sizeof(struct nettlp_msix));
	mnic->rx_irq = calloc(RX_QUEUES,sizeof(struct nettlp_msix));

	mnic->tx_buf = calloc(TX_QUEUES,4096);
	mnic->rx_buf = calloc(RX_QUEUES,4096);

	for(int i=0;i<RX_QUEUES;i++){
		mnic->tx_desc[i] = calloc(DESC_ENTRY_SIZE,sizeof(struct descriptor));
		mnic->rx_desc[i] = calloc(DESC_ENTRY_SIZE,sizeof(struct descriptor));
	}

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
	struct tap_rx_ctl tap_rx_ctl[8];
	struct nettlp_mnic mnic;	
	struct nettlp_msix msix[16];
	cpu_set_t target_cpu_set;
	//pthread_t rx_tid[8]; //tap_read_thread

	memset(&nt,0,sizeof(nt));

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

			nt.requester = nettlp_msg_get_dev_id(host);
			break;
		default:
			usage();
			return -1;
                }
        }

        tap_fd = tap_alloc(ifname);
        if(tap_fd < 0){
                perror("failed to allocate tap");
                return -1;
        }
        
        if(tap_up(ifname) < 0){
                perror("failed to up tap");
                return -1;
        }
        
	memset(&mnic,0,sizeof(mnic));
	mnic.tap_fd = tap_fd;

	mnic_alloc(&mnic);

	for(n=0;n<8;n++){
		mnic.rx_state[n] = RX_STATE_INIT;
	}

	struct nettlp_msix *tx_irq = mnic.tx_irq;
	struct nettlp_msix *rx_irq = mnic.rx_irq;

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

	for(i=0;i<16;i++){
		info("msix addr at %d is %#lx",i,msix[i].addr);
	}
	for(i=0;i<8;i++){
		*tx_irq = msix[i+8];
		*rx_irq = msix[i];
		info("TX IRQ address is %#lx,data is 0x%08x",tx_irq->addr,tx_irq->data);
		info("RX IRQ address is %#lx,data is 0x%08x",rx_irq->addr,rx_irq->data);
		tx_irq++;
		rx_irq++;
	}

	info("Device is %04x",nt.requester);
	info("BAR4 start adress is %#lx",mnic.bar4_start);	       

	tx_irq = mnic.tx_irq;
	rx_irq = mnic.rx_irq;

	for(i=0;i<8;i++){
		//info("TX IRQ address is %#lx,data is 0x%08x",tx_irq->addr,tx_irq->data);
		//info("RX IRQ address is %#lx,data is 0x%08x",rx_irq->addr,rx_irq->data);
		tx_irq++;
		rx_irq++;
	}

	if(signal(SIGINT,signal_handler)==SIG_ERR){
		debug("failed to set signal");
		return -1;
	}


	for(i=0;i<8;i++){
		tap_rx_ctl[i].tap_fd = mnic.tap_fd;
		tap_rx_ctl[i].rx_nt = &mnic.rx_nt;
		tap_rx_ctl[i].rx_state = &mnic.rx_state[i];
		tap_rx_ctl[i].rx_irq = mnic.rx_irq + i;
		tap_rx_ctl[i].rx_desc = mnic.rx_desc[i];
		tap_rx_ctl[i].rxd_ctl = mnic.rx_desc_ctl + i;

		if((ret = pthread_create(&tap_rx_ctl[i].tid,NULL,nettlp_mnic_tap_read_thread,&tap_rx_ctl[i])) != 0){
			debug("%d thread failed to be created",i);
		}

		CPU_ZERO(&target_cpu_set);
		CPU_SET(i,&target_cpu_set);
		pthread_setaffinity_np(tap_rx_ctl[i].tid,sizeof(cpu_set_t),&target_cpu_set);
	}

	info("start nettlp callback");
	memset(&cb,0,sizeof(cb));
	cb.mwr = nettlp_mnic_mwr;
	nettlp_run_cb(nts_ptr,16,&cb,&mnic);

        return 0;
}
