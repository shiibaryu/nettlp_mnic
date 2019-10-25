#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/ip_tunnels.h>

#include <nettlp_mnic_driver.h>
#include "nettlp_msg.h"

#define NETTLP_MNIC_VERSION "0.0.1"
#define DRV_NAME 	    "nettlp_mnic_driver"

#define MNIC_DESC_RING_LEN  1

#define MNIC_TX_QUEUE_ENTRIES 512
#define MNIC_RX_QUEUE_ENTRIES 512

#define PKT_BUF_ENTRY_SIZE 2048

#define TX_CLEAN_BATCH 64
#define POOL_SIZE 64

struct nettlp_mnic_adpter{
	struct pci_devi *pdev;
	struct net_device *dev;
	
	struct mnic_bar0 *bar0;
	struct mnic_bar4 *bar4;
	void *bar2;

	struct tx_queue *txq;
	struct rx_queue *rxq;

	struct pkt_buffer *tx_buf;
	struct pkt_buffer *rx_buf;

	spinlock_t tx_lock;
	spinlock_t rx_lock;

	struct tasklet_struct *rx_tasklet;
	struct napi_struct napi;

#define TX_STATE_READY 1
#define RX_STATE_BUSY  2
	uint32_t tx_state;	
	uint8_t  napi_enabled;
};

//rx_tasklet,xmit,napi,open

static int nettlp_mnic_init(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	//setup coutners
	ndev->tstats = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);	
	if(!ndev->tstats){
		return -ENOMEM;
	}
	
	return 0;
}

static void nettlp_mnic_uninit(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	free_percpu(ndev->tstats);
}

//
static int nettlp_mnic_open(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	struct nettlp_mnic_adpter *m_adpter = netdev_priv(ndev);
	m_adpter->bar4->enabled = 1;
	m_adpter->tx_state = TX_STATE_READY;
	
	//address setting for open
	m_adpter->bar4->tx->tdba = m_adpter->tx_desc_paddr;
	m_adpter->bar4->rx->rdba = m_adpter->rx_desc_paddr;
	
	/*m_adpter->bar4->tx->tdh = m_adpter->tx_desc_paddr;
	m_adpter->bar4->tx->tdh = m_adpter->tx_desc_paddr;

	m_adpter->bar4->tx->tdt = m_adpter->tx_desc_paddr;
	m_adpter->bar4->rx->rdt = m_adpter->rx_desc_paddr;*/
	
	pr_info("notify descriptor base address, TX %#llx,RX %#llx\n",
		m_adpter->bar4->tx->tdba,m_adpter->bar4->rx->rdba);

	return 0;
}

//
static int nettlp_mnic_stop(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	struct nettlp_mnic_adpter *m_adpter = netdev_priv(ndev);
	m_adpter->tx_state = 0;
 	m_adpter->enabled  = 0;

	tasklet_kill(m_adpter->rx_tasklet);

	return 0;
}

static netdev_tx_t nettlp_mnic_xmit(struct sk_buff *skb,struct net_device *dev)
{
	pr_info("%s\n",__func__);
	dma_addr_t dma;
	uint32_t pktlen;
	unsigned long flag;
	struct tx_queue *txq;
	struct nettlp_mnic_adpter *m_adpter = netdev_priv(ndev);

	spin_lock_irqsave(&m_adpter->tx_lock,flag);

	txq = m_adpter->txq;

	if(m_adpter->tx_state != TX_STATE_READY){
		m_adpter->dev->stats.tx_dropped;
		spin_unlock_irqstore(&m_adpter->tx_lock,flag);
		kfree_skb(skb);	
		return NETDEV_TX_OK;
	}

	m_adpter->tx_state = TX_STATE_BUSY;

	pktlen = skb->len;
	dma = dma_map_single(&m_adpter->pdev->dev,skb_mac_header(skb),pktlen,DMA_TO_DEVICE);

	txq->nmtd->read.buffer_addr = dma;
	txq->nmtd->length = pktlen;

	
		
}

static int nettlp_mnic_set_mac(struct net_device *ndev,void *p)
{
	struct nettlp_mnic_adpter *m_adpter = netdev_priv(ndev);	
	struct sockaddr *adddr = p;

	if(!is_valid_ether_addr(addr->sa_data)){
		return -EADDRNOTAVAIL;
	}

	memcpy(ndev->dev_addr,addr->sa_data,ndev->addr_len);
	mnic_get_mac(adpter->bar0->srcmac,ndev->dev_addr);

	return 0;
}

static const struct net_device_ops nettlp_mnic_ops = {
	.ndo_init		= nettlp_mnic_init,
	.ndo_uninit		= nettlp_mnic_uninit,
	.ndo_open		= nettlp_mnic_open, 
	.ndo_stop		= nettlp_mnic_stop,
	.ndo_start_xmit 	= nettlp_mnic_xmit,
	.ndo_get_stats  	= ip_tunnel_get_stats64,
	.ndo_change_mtu 	= eth_change_mtu,
	.ndo_validate_addr 	= eth_validate_addr,
	.ndo_set_mac_address	= nettlp_mnic_set_mac,
};

/*void rx_tasklet(unsigned long tasklet_data)
{
	pr_info("%s\n",__func__);
	unsigned long flags;
	struct sk_buff *skb;
	struct nettlp_mnic_adpter *m_adpter = (struct nettlp_mnic_adpter *)tasklet_data;
	
	spin_lock_irqsave(&m_adpter->rx_lock,flags);
	
}*/

//NAPI Rx polling callback
//@napi: napi polling structure
//budget: count of how many packets we should handle
static int nettlp_mnic_poll(struct napi_structure *napi,int budget)
{
	int ret = 0;
	struct nettlp_mnic_adpter *m_adpter = container_of(napi,struct nettlp_mnic_adpter,napi);
	struct net_device *ndev = m_adpter->dev;
	
	while(){
		
	}

	return 0;
}

static irqreturn_t tx_handler(int irq,void *nic_irq)
{
	pr_info("%s\n",__func__);
	pr_info("tx interrupt irq = %d\n",irq);
	unsigned long flags;
	struct nettlp_mnic_adpter *m_adpter = nic_irq;
	
	spin_lock_irqsave(&m_adpter->tx_lock,flags);
	if(m_adpter->tx_state != TX_STATE_BUSY){
		goto out;
	}
	adpter->tx_state = TX_STATE_READY;

out:	
	spin_unlock_irqrestore(&m_adpter->tx_lock,flags);

	return IRQ_HANDLED;
}

static irqreturn_t rx_handler(int irq,void *nic_irq)
{
	pr_info("%s\n",__func__);
	pr_info("rx interrupt irq = %d\n",irq);
	unsigned long flags;
	struct nettlp_mnic_adpter *m_adpter = nic_irq;

	spin_lock_irqsave(&m_adpter->rx_lock,flags);

	if(m_adpter->rx_napi_enabled){
		m_adpter->rx_napi_enabled = 0;
		napi_schedule(&m_adpter->rx_napi);
	}

	spin_unlock_irqrestore(&m_adpter->rx_lock,flags);

	return IRQ_HANDLED;
}

//use MSI-X
static int nettlp_mnic_interrupts(struct nettlp_mnic_adpter *m_adpter)
{
	//how to register interuppts
	//1.alloc irq vectors
	//2.register irq handler

	int ret,irq;
	
	//1.allocate irq vectors
	ret = pci_alloc_irq_vectors(m_adpter->pdev,2,2,PCI_IRQ_MSIX);
	if(ret < 0){
		pr_info("Request for #%d msix vectors failed, returned %d\n",NETTLP_
NUM_VEC,ret);
		return -1;
	}

	//2.register irq handler(irq1:tx_handler,irq2:rx_handler)
	irq = pci_irq_vector(m_adpter->pdev,0);
	ret = request_irq(irq,tx_handler,0,DRV_NAME,m_adpter);
	if(ret<0){
		pr_info("%s:failed to request irq of tx_handler\n",__func__);
		return -1;
	}

	irq = pci_irq_vector(m_adpter->pdev,1);
	ret = request_irq(irq,rx_handler,1,DRV_NAME,m_adpter);
	if(ret<0){
		pr_info("%s:failed to request irq of rx_handler\n",__func__);
		return -1;
	}

	return 0;
}

static void nettlp_unregister_interrupts(struct nettlp_mnic_adpter *m_adpter)
{
	free_irq(pci_irq_vector(m_adpter->pdev,0),m_adpter);
	free_irq(pci_irq_vector(m_adpter->pdev,1),m_adpter);
} 

static const struct pci_device_id nettlp_mnic_pci_tbl[] = {
	{0x3776,0x8022,PCI_ANY_ID,PCI_ANY_ID,0,0,0},
	{0,}
};
MODULE_DEVICE_TABLE(pci,nettlp_mnic_pci_tbl);

static void nettlp_mnic_pci_init(struct pci_dev *pdev,const struct pci_device_id *ent)
{
	int ret;	
	void *bar0,*bar2,*bar4;
	uint64_t bar0_start,bar0_len;
	uint64_t bar2_start,bar2_len;
	uint64_t bar4_start,bar4_len;
	struct net_device *ndev;
	struct nettlp_mnic_adpter *m_adpter;

	pr_info("%s: register nettlp modern nic device %s\n",
					__func__,pci_name(pdev));

	//see https://bootlin.com/doc/legacy/pci-drivers/pci-drivers.pdf 

	//wake up the device allocate I/O and memory regions of the device
	ret = pci_enable_device(pdev);
	if(ret){
		goto err1;
	}	

	//reserver I/O region
	ret = pci_request_region(pdev,DRV_NAME);
	if(ret){
		goto err2;
	}

	//enable dma 
	pci_set_master(pdev);

	//each pci device have up to 6 I/O or memory regions

	//access the bar0 of the I/O region
	bar0_start = pci_resource_start(pdev,0);
	//access the bar0 of the I/O region size
	bar0_len = pci_resource_len(pdev,0);
	//map the physical address space of the BAR0 in NetTLP adpter to 
	//virtual address space in the kernel
	//get the way to access memory space of bar0
	bar0 = ioremap(bar0_start,bar0_len);

	if(!bar0){
		pr_err("failed to ioremap BAR0 %llx\n",bar0_start);
		goto err3;
	}		
	pr_info("BAR0 %llx is mapped to %p\n",bar0_start,bar0);

	//access the bar2 of the I/O region
	bar2_start = pci_resource_start(pdev,2);
	//access the bar2 of the I/O region size
	bar2_len = pci_resource_len(pdev,2);
	//map the physical address space of the BAR2 in NetTLP adpter to 
	//virtual address space in the kernel
	//get the way to access memory space of bar2
	bar2 = ioremap(bar2_start,bar2_len);

	if(!bar2){
		pr_err("failed to ioremap BAR2 %llx\n",bar2_start);
		goto err4;
	}		
	pr_info("BAR2 %llx is mapped to %p\n",bar2_start,bar2);

	//access the bar4 of the I/O region
	bar4_start = pci_resource_start(pdev,4);
	//access the bar4 of the I/O region size
	bar4_len = pci_resource_len(pdev,4);
	//map the physical address space of the BAR4 in NetTLP adpter to 
	//virtual address space in the kernel
	//get the way to access memory space of bar4
	bar4 = ioremap(bar4_start,bar4_len);

	if(!bar4){
		pr_err("failed to ioremap BAR4 %llx\n",bar4_start);
		goto err5;
	}		
	pr_info("BAR4 %llx is mapped to %p\n",bar4_start,bar4);

	ret = -ENOMEM;
	//allocate ethernet device(struct net_device) and register
	ndev = allocate_etherdev(sizeof(*m_adpter));
	if(!ndev){
		goto err6;
	}
	//associate with the device private data
	SET_NETDEV_DEV(ndev,&pdev->dev);
	pci_set_drvdata(pdev,ndev);

	//access network device private data
	//like getting a pointer of net_device
	m_adpter = (struct nettlp_mnic_adpter *)netdev_priv(ndev);
	m_adpter->dev = dev;
	m_adpter->bar0 = bar0;
	m_adpter->bar2 = bar2;	
	m_adpter->bar4 = bar4;

	//decreare and device with more ore less than 32bit-bus master capability
	ret = pci_set_dma_mask(pdev,DMA_32BIT_MASK);
	if(!ret){
		ret = pci_set_cosistent_dma_mask(pdev,DMA_32BIT_MASK);
	}
	if(ret){
		pr_info("No suitable DMA available.\n");
		goto err6;
	}

	//get a virtual address of dma buffer and store physical address to variable 3
	m_adpter->txq = dma_alloc_coherent(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,&m_adpter->tx_desc_paddr,GFP_KERNEL);
	if(!m_adpter->txq){
		pr_info("%s: failed to alloc tx descriptor\n",__func__);
		goto err6;
	}	

	m_adpter->rxq = dma_alloc_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,&m_adpter->rx_desc_paddr,GFP_KERNEL);
	if(!m_adpter->rxq){
		pr_info("%s: failed to alloc rx descriptor\n",__func__);
		goto err6;
	}	

	m_adpter->tx_buf = dma_alloc_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,&adpter->tx_buf->paddr,GFP_KERNEL);
	if(!m_adpter->tx_buf){
		pr_info("%s: failed to alloc tx buffer\n",__func__);
		goto err6;
	}	

	m_adpter->rx_buf = dma_alloc_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,&adpter->rx_buf->paddr,GFP_KERNEL);
	if(!m_adpter->rx_buf){
		pr_info("%s: failed to alloc rx buffer\n",__func__);
		goto err6;
	}	

	spin_lock_init(&adpter->tx_lock);
	spin_lock_init(&adpter->rx_lock);

	mnic_get_mac(ndev->dev_addr,adpter->bar0->srcmac);	
	ndev->needs_free_netdev = true;
	ndev->netdev_ops = &nettlp_mnic_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_MAX_MTU;
	
	ret = register_netdev(ndev);
	if(ret){
		goto err7;
	}
	
	//initialize tasklet for rx interrupt
	adpter->rx_tasklet = kmalloc(sizeof(struct tasklet_struct),GFP_KERNEL);
	if(m_adpter->rx_tasklet){
		ret = -ENOMEM;
		goto err8;
	}
	tasklet_init(m_adpter->rx_tasklet,rx_tasklet,(unsigned long)m_adpter);

	//register interrupt
	ret = nettlp_register_interrupts(m_adpter);
	if(ret){
		goto err9;
	}

	nettlp_msg_init(bar4_start,PCI_DEVID(pdev->bus->number,pdev->devfn));

	//initialize and enable napi
	netif_napi_add(ndev,&m_adpter->napi,nettlp_mnic_poll,POOL_SIZE);
	napi_enable(&m_adpter->napi);
	m_adpter->napi_enabled = 1;

	//initialize each variable
	m_adpter->txq->tx_index = 0;
	m_adpter->txq->clean_index = 0;
	m_adpter->txq->num_entries = 0;
	
	m_adpter->rxq->rx_index = 0;
	m_adpter->rxq->num_entries = 0;
	
	m_adpter->tx_buf->index = 0;
	m_adpter->tx_buf->memp->index = 0;

	m_adpter->rx_buf->index = 0;
	m_adpter->rx_buf->memp->index = 0;

	m_adpter->bar4->tx->tdh = 0;
	m_adpter->bar4->tx->tdt = 0;
	m_adpter->bar4->rx->rdh = 0;
	m_adpter->bar4->rx->rdt = 0;

	pr_info("%s: probe finished.",__func__);
	
	return 0;
err9:
	kfree(m_adapter->rx_tasklet);
err8:
	unregister_netdev(dev);
err7:
	dma_free_coherent(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,(void*)m_adpter->txq,m_adpter->txq->tx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,(void*)m_adpter->rxq,m_adpter->rxq->rx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->tx_buf,&m_adpter->tx_buf->paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->rx_buf,&m_adpter->rx_buf->paddr);

err6:
	iounmap(bar4);
err5:
	iounmap(bar2);
err4:
	iounmap(bar0);
err3:
	pci_release_regions(pdev);
err2:
	pci_disable_device(pdev);
err1:

	return ret;
}


static void nettlp_mnic_pci_remove(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct nettl_snic_adpter *m_adpter = netdev_priv(dev);

	pr_info("start remove pci config");

	kfree(m_adpter->rx_tasklet);
	
	nettlp_msg_fini();
	nettlp_unregister_interrupts(m_adpter);
	pci_free_irq_vectors(pdev);
	
	unregister_netdev(dev);
	
	dma_free_coherent(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,(void*)m_adpter->txq,m_adpter->txq->tx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,(void*)m_adpter->rxq,m_adpter->rxq->rx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->tx_buf,&m_adpter->tx_buf->paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->rx_buf,&m_adpter->rx_buf->paddr);
	
	iounmap(bar4);
	iounmap(bar2);
	iounmap(bar0);
	
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	
	return;
}

struct pci_driver nettlp_mnic_pci_driver = {
	.name = DRV_NAME,
	.id_table = nettlp_mnic_pci_tbl,
	.probe = nettlp_mnic_pci_init,
	.remove = nettlp_mnic_pci_remove,
};

static int __init nettlp_mnic_module_init(void)
{
	pr_info("nettlp_mnic is loaded\n");

	return pci_register_driver(&nettlp_mnic_pci_driver);
}
module_init(nettlp_mnic_module_inic);

static void __exit nettlp_mnic_module_exit(void)
{
	pci_unregister_driver(&nettlp_mnic_pci_driver);
	pr_info("nettlp_mnic is unloaded\n");

	return;
}
module_exit(nettlp_mnic_module_exit);

MODULE_AUTHOR("Ryusei Shiiba <siiba@sfc.wide.ad.jp>");
MODULE_DECRIPTION("nettlp_mnic");
MODULE_LICENSE("GPL");
MODULE_VERSION(NETTLP_MNIC_VERSION);
