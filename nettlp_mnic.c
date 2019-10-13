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

void usage()
{
}

int main(int argc,char **argv)
{
        int opt,ret,fd;
        char *ifname = "tap8";
	struct nettlp nt;
	struct in_addr host;
	struct mnic mnic;	

	memset(&nt,0,sizeof(nt));

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

        fd = tap_alloc(ifname);
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
	
	memset(&mnic,0,sizeof(mnic));
	snic.fd = fd;
	snic.bar4_start = nettlp_msg_get_bar4_start(host);
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

       
        return 0;
}
