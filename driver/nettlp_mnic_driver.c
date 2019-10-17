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
struct nettlp_mnic_adpter{
	struct pci_devi *pdev;
	struct net_device *dev;
	
	struct mnic_bar0 *bar0;
	struct mnic_bar4 *bar4;
	void *bar2;

	struct tx_queue *txq;
	struct rx_queue *rxq;
};

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
	adpter->tx_desc = dma_alloc_coherent(&pdev->dev,sizeof(struct descriptor)*MNIC_DESC_RING_LEN,&adpter->tx_desc_paddr,GFP_KERNEL);
	if(!adpter->tx_desc){
		pr_info("%s: failed to alloc tx descriptor\n",__func__);
		goto err6;
	}	

	adpter->rx_desc = dma_alloc_coherent(&pdev->dev,sizeof(struct descriptor)*MNIC_DESC_RING_LEN,&adpter->rx_desc_paddr,GFP_KERNEL);
	if(!adpter->rx_desc){
		pr_info("%s: failed to alloc rx descriptor\n",__func__);
		goto err6;
	}	

	
}

static void nettlp_mnic_pci_remove(struct pci_dev *pdev)
{

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
