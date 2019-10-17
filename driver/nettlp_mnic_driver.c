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

struct nettlp_mnic_adpter{
	struct pci_devi *pdev;
	struct net_device *dev;
	
	struct mnic_bar0 *bar0;
	struct mnic_bar4 *bar4;
	
	struct tx_queue *txq;
	struct rx_queue *rxq;
};

static const struct pci_device_id nettlp_mnic_pci_tbl[] = {
	{0x3776,0x8022,PCI_ANY_ID,PCI_ANY_ID,0,0,0},
	{0,{
};
MODULE_DEVICE_TABLE(pci,nettlp_mnic_pci_tbl);

static void nettlp_mnic_pci_init(struct pci_dev *pdev)
{
	
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
