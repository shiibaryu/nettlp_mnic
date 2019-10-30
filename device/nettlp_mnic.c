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

struct mnic_device{
	//tap fd 
	int tap_fd;
	uintptr_t bar4_start;
	uintptr_t tx_desc_base;
	uintptr_t rx_desc_base;

	struct nettlp *rx_nt; 
	struct nettlp_msix tx_irq,rx_irq;

	struct mnic_rx_descriptor *rx_desc;
	struct mnic_tx_descriptor *tx_desc;

};

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

int nettlp_mnic_mwr(struct nettlp *nt,struct tlp_mr_hdr *mr,void *m,size_t count,void *arg)
{
	int ret;
	struct nettlp_mnic *mnic = arg;
	
	dma_addr = tlp_mr_addr(mh);
	info("dma addr is %lx\n",mh);
	
	if(is_mwr_addr_tx_desc_ptr(mnic->bar4_start,dma_addr)){
		memcpy(&mnic->tx_desc_base,m,8);
		info("TX desc base is %lx",mnic->tx_desc_base);
	}
	else if(is_mwr_addr_rx_desc_ptr(mnic->bar4_start,dma_addr)){
		memcpy(&mnic->rx_desc_base,m,8);
		info("RX desc base is %lx",mnic->rx_desc_base);
	}
	else if(is_mwr_addr_tx_tail_ptr(mnic->bar4_start,dma_addr)){
		if(mnic->tx_desc_base == 0){
			debug("tx desc base is NULL");
			goto tx_end;
		}

	}
	else if(is_mwr_addr_rx_tail_ptr(mnic->bar4_start,dma_addr)){
		if(mnic->rx_desc_base == 0){
			debug("rx desc base is NULL");
			goto rx_end;
		}

	}
}

void *nettlp_mnic_tap_read_thread(void *arg)
{
	int ret,len;
	char buf[2048];
	struct nettlp_mnic *mnic = arg;
	struct poll_fd x[1] = {{.fd = mnic->tap_fd,.events = POLLIN}};

	while(1){
		if(caught_signal){
			break;
		}
		ret = poll(x,1,500);
		if(ret < 0 || ret == 0 || !(x[0].revents & POLLIN)){
			continue;
		}
		
		len = read(mnic->tap_fd,buf,sizeof(buf));
		if(len<0){
			perror("read");
			continue;
		}
		
		ret = dma_write(mnic->,mnic->rx_desc.addr,buf,len);
		if(ret < 0){
			debug("failed to dma_write pkt to %lx",mnic->rx_desc.addr);
			continue;
		}
		
		info("RX done. DMA write to %lx %d byte",mnic->rx_desc,len);
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
        int opt,ret,tap_fd;
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
		
	mnic.tx_desc = malloc(sizeof(struct mnic_tx_descriptor)*TX_QUEUE_ENTRIES);
	mnic.rx_desc = malloc(sizeof(struct mnic_rx_descriptor)*RX_QUEUE_ENTRIES);
	memset(mnic.tx_desc,0,sizeof(struct mnic_tx_descriptor);
	memset(mnic.rx_desc,0,sizeof(struct mnic_rx_descriptor);


	mnic.fd = tap_fd;
	mnic.bar4_start = nettlp_msg_get_bar4_start(host);	
	if(snic.bar4_start == 0){
		debug("failed to get BAR4 addr from %s\n",inet_ntoa(host));
		info("nettlp_msg_get_bar4_start");
		return -1;
	}

	ret = nettlp_msg_get_msix_table(host,msix,2);
	if(ret < 0){
		debug("faled to get msix table from %s\n",inet_ntoa(host));
		info("nettlp_msg_get_msix_table");
	}	

	mnic.tx_irq = msix[0];
	mnic.rx_irq = msix[1];

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
