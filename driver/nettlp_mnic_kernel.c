#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/pci.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/ip_tunnels.h>

#include <nettlp_mnic_driver.h>
#include "nettlp_msg.h"

#define NETTLP_MNIC_VERSION "0.0.1"
#define DRV_NAME 	    "nettlp_mnic_driver"

#define MNIC_DESC_RING_LEN  1

//#define TX_QUEUE_ENTRIES 512
//#define RX_QUEUE_ENTRIES 512

//#define PKT_BUF_ENTRY_SIZE 2048

#define TX_CLEAN_BATCH 64
#define BATCH_SIZE 68


/*struct nettlp_mnic_adpter{
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
};*/

//xmit,rx_poll,open

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

/*
static int nettlp_mnic_open(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	struct nettlp_mnic_adpter *m_adpter = netdev_priv(ndev);
	m_adpter->bar4->enabled = 1;
	m_adpter->tx_state = TX_STATE_READY;
	
	//address setting for open
	m_adpter->bar4->tx->tdba = m_adpter->tx_desc_paddr;
	m_adpter->bar4->rx->rdba = m_adpter->rx_desc_paddr;
	
	m_adpter->bar4->tx->tdh = m_adpter->tx_desc_paddr;
	m_adpter->bar4->tx->tdh = m_adpter->tx_desc_paddr;

	m_adpter->bar4->rx->rdt = m_adpter->rx_desc_paddr;
	m_adpter->bar4->rx->rdt = m_adpter->rx_desc_paddr;
	
	r_info("notify descriptor base address, TX %#llx,RX %#llx\n",
		m_adpter->bar4->tx->tdba,m_adpter->bar4->rx->rdba);

	return 0;
}
*/

static void mnic_clean_tx_ring(struct mnic_ring *tx_ring)
{
	uint16_t i = tx_ring->next_to_clean;
	struct mnic_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];

	while (i != tx_ring->next_to_use) {
		struct mnic_adv_tx_desc *eop_desc, *tx_desc;

		/* Free all the Tx ring sk_buffs */
		dev_kfree_skb_any(tx_buffer->skb);

		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
				 dma_unmap_addr(tx_buffer, dma),
				 dma_unmap_len(tx_buffer, len),
				 DMA_TO_DEVICE);

		/* check for eop_desc to determine the end of the packet */
		eop_desc = tx_buffer->next_to_watch;
		tx_desc = MNIC_TX_DESC(tx_ring, i);

		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(i == tx_ring->count)) {
				i=0;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = MNIC_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len)){
				dma_unmap_page(tx_ring->dev,
					       dma_unmap_addr(tx_buffer, dma),
					       dma_unmap_len(tx_buffer, len),
					       DMA_TO_DEVICE);
			}
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		i++;
		if (unlikely(i == tx_ring->count)) {
			i = 0;
			tx_buffer = tx_ring->tx_buffer_info;
		}
	}

	//netdev_tx_reset_queue(txring_txq(tx_ring));

	/* reset next_to_use and next_to_clean */
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

void mnic_free_tx_resource(struct mnic_ring *tx_ring)
{
	mnic_clean_tx_ring(tx_ring);
	
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	
	if(tx_ring->desc){
		dma_free_coherent(tx_ring->dev,tx_ring->size,tx_ring->desc,tx_ring_dma);
		tx_ring->desc = NULL;
	}
	else{
		return;
	}
}

static void mnic_free_all_tx_resources(struct mnic_adapter *adapter)
{
	int i;
	
	for(i=0;i<adapter->num_tx_queues;i++){
		if(adapter->tx_ring[i]){
			mnic_free_tx_resource(adapter->tx_ring[i]);
		}
		else{
			pr_info("%s: no tx ring\n",__func__);
		}
	}
}
static int mnic_setup_tx_resource(struct mnic_ring *tx_ring)
{
	int size;
	struct device *dev = tx_ring->dev;

	size = sizeof(struct mnic_tx_buffer)*tx_ring->count;
	
	tx_ring->tx_buffer_info = vmalloc(size);
	if(!tx_ring->tx_buffer_info){
		pr_info("%s: failed to alloc tx bufffer\n",__func__);
		goto err;
	}
	
	tx_ring->size = tx_ring->count*sizeof(struct mnic_adv_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size,4096);
	
	tx_ring->desc = dma_alloc_coherent(dev,tx_ring->size,&tx_ring->dma,GFP_KERNEL);
	if(!tx_ring->desc){
		goto err;
	}
	
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	dev_err("failed to allocate memory for tx descriptor ring\n");
	return -ENOMEM;	
}


static int mnic_setup_all_tx_resources(struct mnic_adapter *adapter)
{	
	int i,ret = 0;

	for(i=0; i < adapter->num_tx_queues; i++){
		ret = mnic_setup_tx_resource(adapter->tx_ring[i]);
		if(ret){
			pr_info("%s: failed to setup tx_resource\n",__func__);
			for(i--;i>0;i--){
				mnic_free_tx_resources(adapter->tx_ring[i]);
			}
			break;
		}
	}
	
	return ret;
}

static void mnic_clean_rx_ring(struct mnic_ring *rx_ring)
{
	uint16_t i = rx_ring->next_to_clean;
	dev_kfree_skb(rx_ring->skb);
	rx_ring->skb = NULL;

	/* Free all the Rx ring sk_buffs */
	while (i != rx_ring->next_to_alloc) {
		struct mnic_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];

		/* Invalidate cache lines that may have been written to by
		 * device so that we avoid corrupting memory.
		 */
		dma_sync_single_range_for_cpu(rx_ring->dev,
					      buffer_info->dma,
					      buffer_info->page_offset,
					      //やばいかも？
					      2048,
					      DMA_FROM_DEVICE);

		/* free resources associated with mapping */
		dma_unmap_page_attrs(rx_ring->dev,
				     buffer_info->dma,
				     //やばいかも
				     2048,
				     DMA_FROM_DEVICE,
				     IGB_RX_DMA_ATTR);
		__page_frag_cache_drain(buffer_info->page,
					buffer_info->pagecnt_bias);

		i++;
		if (i == rx_ring->count){
			i = 0;
		}
	}

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

void mnic_free_rx_resources(struct mnic_ring *rx_ring)
{
	mnic_clean_rx_ring(rx_ring);

	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->size,
			  rx_ring->desc, rx_ring->dma);
	rx_ring->desc = NULL;
}

int mnic_setup_rx_resource(struct mnic_ring *rx_ring)
{
	int size;
	struct device *dev = rx_ring->dev;

	size = sizeof(struct mnic_rx_buffer) * rx_ring->count;

	rx_ring->rx_buffer_info = vmalloc(size);

	if (!rx_ring->rx_buffer_info){
		goto err;
	}

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(union mnic_adv_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);
	rx_ring->desc = dma_alloc_coherent(dev, rx_ring->size,
					   &rx_ring->dma, GFP_KERNEL);

	if (!rx_ring->desc){
		goto err;
	}

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;

err:
	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");

	return -ENOMEM;
}

static int mnic_setup_all_rx_resources(struct mnic_adapter *adapter)
{
	int i,ret=0;
	
	for(i=0;i<adapter->num_rx_queues;i++){
		ret = mnic_setup_rx_resouce(adapter->rx_ring[i]);
		if(ret){
			pr_info("%s: failed to set up rx resouce\n",__func__);
			for(i--;i>0;i--){
				mnic_free_rx_resources(adapter->rx_ring[i]);
			}
			break;
		}
	}

	return err;
}

static irqreturn_t mnic_msix_ring(int irq,void *data)
{
	struct mnic_q_vector *q_vector = (struct mnic_q_vector *)data;
	
	napi_schedule(&q_vector->napi);

	return IRQ_HANDLED;
}

static int mnic_request_msix(struct mnic_adapter *adapter)
{
	int i,ret=0;
	int vector=0;
	int free_vector = 0;
	struct net_device *ndev = adapter->ndev;

	/*ret = request_irq(adpter->msix_entries[vector].vector,
			igb_msix_other,0,ndev->name,adpter);*/

	for(i=0;i<adapter->num_q_vectors;i++){
		struct mnic_q_vector *q_vector = adapter->q_vector[i];
		//vector++;

		if(q_vector->rx.ring && q_vector->tx.ring){
			sprintf(q_vector->name, "%s-TxRx-%u", netdev->name,
				q_vector->rx.ring->queue_index);
		}
		else if (q_vector->tx.ring){
			sprintf(q_vector->name, "%s-tx-%u", netdev->name,
				q_vector->tx.ring->queue_index);
		}
		else if (q_vector->rx.ring){
			sprintf(q_vector->name, "%s-rx-%u", netdev->name,
				q_vector->rx.ring->queue_index);
		}
		else{
			sprintf(q_vector->name, "%s-unused", netdev->name);
		}

		ret = request_irq(adpter->msix_entries[vector].vector,mnic_msix_ring,0,q_vector->name,q_vector);
		if(ret){
			goto err_free;
		}
	}
	
	//mnic_configure_msix(adpter);

	return 0;

err_free:

	/* free already assigned IRQs */
	free_irq(adapter->msix_entries[free_vector++].vector, adapter);
	vector--;

	for (i = 0; i < vector; i++) {
		free_irq(adapter->msix_entries[free_vector++].vector,
			 adapter->q_vector[i]);
	}
	return ret;
}

static int mnic_request_irq(struct mnic_adapter *adapter)
{
	int ret;
	struct net_device *ndev = adapter->ndev;
	struct pci_dev *pdev = adapter->pdev;
	
	ret = mnic_request_msix(adapter);

	if(ret==0){
		//うまくいけばここでおわり
		goto request_done;
	}
	else{
		pr_info("%s: koko ni kiteha dame!!!!!");
		pr_info("%s: outihe okeeri!!!");
		return -1;
		/*mnic_free_all_tx_resources(adpter);
		mnic_free_all_rx_resources(adpter);

		mnic_clear_interrupt_scheme(adpter);
		ret = mnic_init_interrupt_scheme(adpter,false);
		if(ret){
			goto request_done;
		}	
	
		mnic_setup_all_tx_resources(adpter);
		mnic_setup_all_rx_resources(adpter);
		mnic_configure(adpter);

		//mnic_assign_vector(adpter->q_vector[0],0);

		ret = request_irq(pdev->irq,mnic_itr,IRQ_SHARED,ndev->name,adpter):
		if(ret){
			pr_info("%s: failed to get irq\n",__func__);
		}*/
	}

request_done:
	return ret;
}

static int __mnic_open(struct net_device *ndev,bool resuming)
{
	int ret,i;
	struct mnic_adapter *adapter = netdev_priv(ndev);
	struct pci_dev *pdev = adapter->pdev;

	/*if(!resuming){
		pm_runtime_get_sync(&pdev->dev);
	}*/

	netif_carrier_off(ndev);
	
	/* allocate transmit descriptors*/
	ret = mnic_setup_all_tx_resources(adapter);
	if(ret){
		goto err_setup_tx;
	}

	/* allocate receive descriptors*/
	ret = mnic_setup_all_rx_resources(adapter);
	if(ret){
		goto err_setup_rx;
	}

	//call mnic_desc_unused
	//mnic_alloc_rx_buffers(ring,mnic_desc_unused(ring));
	
	ret = mnic_request_irq(adapter);
	if(ret){
		goto err_req_irq;
	}

	/* Notify the stack of the actual queue counts.*/
	ret = netif_set_real_num_tx_queues(adapter->ndev,adapter->num_tx_queues);
	if(ret){
		goto err_set_queues;
	}

	ret = netif_set_real_num_rx_queues(adapter->ndev,adapter->num_rx_queues);
	if(ret){
		goto err_set_queues;
	}

	for(i=0;i<adapter->num_q_vectors;i++){
		napi_enable(&(adapter->q_vector[i]->napi));
	}

	//mnic_irq_enable(adpter);
	netif_tx_start_all_queues(ndev);
	
	/*if(!resuming){
		pm_runtime_put(&pdev->dev);
	}*/

	return 0;

err_set_queues:
	pr_info("%s:err",__func__);
	mnic_free_irq(adapter);

err_req_irq:
	pr_info("%s:err",__func__);
	/*mnic_release_hw_control(adapter);
	mnic_power_down_link(adapter);*/
	mnic_free_all_rx_resources(adapter);

err_setup_rx:
	pr_info("%s:err",__func__);
	igb_free_all_tx_resources(adapter);

err_setup_tx:
	//igb_reset(adapter);
	pr_info("%s:igb_reset\n",__func__);

	return ret;
}

static int mnic_open(struct net_device *ndev)
{
	return __mnic_open(ndev,false);
}

static int nettlp_mnic_stop(struct net_device *ndev)
{
	pr_info("%s\n",__func__);

	struct nettlp_mnic_adapter *m_adapter = netdev_priv(ndev);
	m_adapter->tx_state = 0;
 	m_adapter->enabled  = 0;

	tasklet_kill(m_adapter->rx_tasklet);

	return 0;
}

static netdev_tx_t mnic_xmit_frame_ring(struct sk_buff *skb,struct net_device *dev)
{
	pr_info("%s\n",__func__);
	struct mnic_tx_buffer *first;
	uint32_t flags;
	uint32_t count = TXD_USE_COUNT(skb_headlen(skb));
	uint8_t hdr_len = 0;	
	
}

static int nettlp_mnic_set_mac(struct net_device *ndev,void *p)
{
	struct nettlp_mnic_adapter *m_adapter = netdev_priv(ndev);	
	struct sockaddr *addr = p;

	if(!is_valid_ether_addr(addr->sa_data)){
		return -EADDRNOTAVAIL;
	}

	memcpy(ndev->dev_addr,addr->sa_data,ndev->addr_len);
	mnic_get_mac(adapter->bar0->srcmac,ndev->dev_addr);

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

void rx_tasklet(unsigned long tasklet_data)
{
	pr_info("%s\n",__func__);
	unsigned long flags;
	struct sk_buff *skb;
	struct nettlp_mnic_adpter *m_adapter = (struct nettlp_mnic_adapter *)tasklet_data;
	
	spin_lock_irqsave(&m_adapter->rx_lock,flags);

	//koko
	dma_unmap_single(&m_adapter->pdev->dev,m_adapter->,2048,DMA_FROM_DEVICE);

	skb = netdev_alloc_skb_ip_align(m_adapter->ndev,);
	if(!skb){
		m_adapter->ndev->stats.rx_dropped++;
		pr_info("%s: failed to alloc skb\n",__func__);
		goto out;
	}		

	skb_copy_to_liner_data_offset(skb,NET_IP_ALIGN,
				      m_adapter->rx_buf,
				      m_adapter->rx_buf->size);

	skb_put(skb,m_adapter->rx_desc->size);	
	skb->protocol = eth_type_trans(skb,m_adapter->ndev);
	skb->ip_summed = CHECKSUM_NONE;
	
	netif_rx(skb);
	m_adapter->ndev->stats.rx_packets++;
	//koko
	m_adapter->ndev->stats.rx_bytes += ;

	//koko
	m_adapter->rx_desc->addr = m_adapter->rx_buf_paddr;
	m_adapter->bar4->rx_desc_idx = m_adapter->rx_desc_idx;

out:
	spin_unlock_irqrestore(&m_adpter->rx_lock,flags);

	return;
}

//NAPI Rx polling callback
//@napi: napi polling structure
//budget: count of how many packets we should handle
static int nettlp_mnic_poll(struct napi_structure *napi,int budget)
{
	int done = 0;
	struct nettlp_mnic_adpter *m_adapter = container_of(napi,struct nettlp_mnic_adpter,napi);
	struct net_device *ndev = m_adapter->dev;
	
	while(done < budget){
		if(){
			break;
		}
		else{
			tasklet_schedule(m_adapter->rx_tasklet);
			done++;			
		}	
	}

	if(done < budget){
		napi_complete(napi);
		m_adapter->napi_enabled = 1;
	}

	return done;
}

static irqreturn_t tx_handler(int irq,void *nic_irq)
{
	pr_info("%s\n",__func__);
	pr_info("tx interrupt irq = %d\n",irq);
	unsigned long flags;
	struct nettlp_mnic_adpter *m_adapter = nic_irq;
	
	spin_lock_irqsave(&m_adapter->tx_lock,flags);
	if(m_adapter->tx_state != TX_STATE_BUSY){
		goto out;
	}
	adapter->tx_state = TX_STATE_READY;

out:	
	spin_unlock_irqrestore(&m_adapter->tx_lock,flags);

	return IRQ_HANDLED;
}

static irqreturn_t rx_handler(int irq,void *nic_irq)
{
	pr_info("%s\n",__func__);
	pr_info("rx interrupt irq = %d\n",irq);
	unsigned long flags;
	struct nettlp_mnic_adpter *m_adapter = nic_irq;

	spin_lock_irqsave(&m_adapter->rx_lock,flags);

	if(m_adapter->rx_napi_enabled){
		m_adapter->rx_napi_enabled = 0;
		napi_schedule(&m_adapter->rx_napi);
	}

	spin_unlock_irqrestore(&m_adapter->rx_lock,flags);

	return IRQ_HANDLED;
}

//use MSI-X
/*static int nettlp_mnic_interrupts(struct nettlp_mnic_adpter *m_adpter)
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
}*/

static void nettlp_unregister_interrupts(struct nettlp_mnic_adapter *m_adapter)
{
	free_irq(pci_irq_vector(m_adapter->pdev,0),m_adapter);
	free_irq(pci_irq_vector(m_adapter->pdev,1),m_adapter);
} 

static const struct pci_device_id mnic_pci_tbl[] = {
	{0x3776,0x8022,PCI_ANY_ID,PCI_ANY_ID,0,0,0},
	{0,}
};
MODULE_DEVICE_TABLE(pci,nettlp_mnic_pci_tbl);

static void mnic_reset_interrupt_capability(struct igb_adapter *adapter)
{
	pci_disable_msix(adapter->pdev);
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;
}

static void mnic_free_q_vector(struct mnic_adapter *adapter, int v_idx)
{
	struct mnic_q_vector *q_vector = adapter->q_vector[v_idx];

	if (q_vector->tx.ring){
		adapter->tx_ring[q_vector->tx.ring->queue_index] = NULL;
	}

	if (q_vector->rx.ring){
		adapter->tx_ring[q_vector->rx.ring->queue_index] = NULL;
	}

	adapter->q_vector[v_idx] = NULL;
	netif_napi_del(&q_vector->napi);

	kfree_rcu(q_vector, rcu);
}

static void mnic_free_q_vectors(struct mnic_adapter *adapter)
{
	int v_idx = adapter->num_q_vectors;

	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;	

	adapter->num_q_vectors = 0;

	while (v_idx--){
		mnic_free_q_vector(adapter, v_idx);
	}

}
static void mnic_clear_interrupt_scheme(struct mnic_adapter *adapter)
{
	mnic_free_q_vectors(adapter);
	mnic_reset_interrupt_capability(adapter);
}

static void mnic_set_interrupt_capability(struct mnic_adpter *adapter,bool msix)
{
	int ret;
	int numvecs,i;

	//number of q for tx and rx(currently 8 or num_cpu_core,but maybe 16 good)
	if(adapter->rss_queues > MAX_MSIX_ENTRIES){
		pr_info("%s: adpter->rss_queus over MAX MSIX ENTRIES");
	}

	adapter->num_rx_queues = adapter->rss_queues;
	adapter->num_tx_queues = adapter->rss_queues;

	numvecs = adapter->num_rx_queues;
	//もしかしたら足しちゃダメかも
	//IXGBEはmax 16,igbとこれは8
	numvecs += adapter->num_tx_queues;

	adapter->num_q_vectors = numvecs;

	//add 1 vector for link status interrupts
	numvecs++;
	adapter->msix_entries = kcalloc(numvecs,sizeof(struct msix_entry),GFP_KERNEL);
	
	for(i=0;i<numvecs;i++){
		adapter->msix_entries[i].entry = i;	
	}
	
	ret = pci_enable_msix(adapter->pdev,adapter->msix_entries,numvecs);

	/*If success, return*/	
	if(ret==0){
		return;
	}

	mnic_reset_interrupt_capability(adapter);
}


static void mnic_add_ring(struct mnic_ring *ring,struct mnic_ring_container *head)
{
	head->ring = ring;
	head->count++;
}

static int minc_alloc_q_vector(struct mnic_adapter *adapter,int v_count,int v_idx,int txr_count,int txr_idx,int rxr_count,int rxt_idx)
{
	struct mnic_q_vector *q_vector;
	struct mnic_ring *ring;
	int ring_count,size;
	
	if(txr_count > 1 || rxr_count > 1){
		return -ENOMEM;
	}

	ring_count = txr_count + rxr_count;
	size = sizeof(struct mnic_q_vector)+(sizeof(struct mnic_ring)*ring_count);
	
	q_vector = kzalloc(size,GFP_KERNEL);
	if(!q_vector){
		return -ENOMEM;
	}

	netif_napi_add(adapter->ndev,&q_vector->napi,mnic_poll,64);

	adapter->q_vector[v_idx] = q_vector;
	q_vector->adapter = adapter;

	q_vector->tx.work_limit = adapter->tx_work_limit;

	/*
	q_vector->itr_register = adpter->hw.hw_addr + E1000_EINTR(0);
  	q_vector->itr_val      = IGB_START_ITR;		
	i*/
	ring = q_vector->ring;
	

	/* intialize ITR */
	if (rxr_count) {
		/* rx or rx/tx vector */
		if (!adapter->rx_itr_setting || adapter->rx_itr_setting > 3)
			q_vector->itr_val = adapter->rx_itr_setting;
	} 
	else {
		/* tx only vector */
		if (!adapter->tx_itr_setting || adapter->tx_itr_setting > 3)
			q_vector->itr_val = adapter->tx_itr_setting;
	}
	
	if (txr_count) {
		/* assign generic ring traits */
		ring->dev = &adapter->pdev->dev;
		ring->ndev = adapter->ndev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Tx values */
		mnic_add_ring(ring, &q_vector->tx);

		/* apply Tx specific ring traits */
		ring->count = adapter->tx_ring_count;
		ring->queue_index = txr_idx;

		u64_stats_init(&ring->tx_syncp);
		u64_stats_init(&ring->tx_syncp2);

		/* assign ring to adapter */
		adapter->tx_ring[txr_idx] = ring;

		/* push pointer to next ring */
		ring++;
	}

	if(rxr_count){
		ring->dev = &adapter->pdev->dev;
		ring->ndev = adapter->ndev;

		ring->q_vector = q_vector;
		mnic_add_ring(ring,&q_vector->rx);
	
		ring->count = adpter->rx_ring_count;
		ring->queue_index = rxr_idx;
	
		u64_stats_init(&ring->rx_syncp);

		adpter->rx_ring[rxr_idx] = ring;
	}

	return 0;
}

//allocate memory for interrupt vectors
static int mnic_alloc_q_vectors(struct mnic_adapter *adapter)
{
	int q_vectors = adapter->num_q_vectors;
	int rxr_remaining = adapter->num_rx_queues;
	int txr_remaining = adapter->num_tx_queues;
	int rxr_idx = 0, txr_idx = 0, v_idx = 0;
	int ret;
	
	if(q_vectors >= (rxr_remaning + txr_remaining)){
		for(;rxr_remaining;v_idx++){
			ret = mnic_alloc_q_vector(adapter,q_vectors,v_idx,0,0,1,rxr_idx);
			if(ret){
				goto err_out;
			}
			rxr_remaining--;
			rxr_idx++;
		}
	}

	for(;v_idx < q_vectors;v_idx++){
		int rqpv = DIV_ROUND_UP(rxr_remaining,q_vectors - v_idx);
		int tqpv = DIV_ROUND_UP(txr_remaining,q_vectors - v_idx);
		ret = mnic_alloc_q_vector(adapter,q_vectors,v_idx,tqpv,txr_idx,rpqv,rxr_idx);
		if(ret){
			goto err_out;
		}
		
		rxr_remaining -= rqpv;
		txr_remaining -= tqpv;
		rxr_idx++;	
		txr_idx++;
	}

	return 0;

err_out:
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--){
		igb_free_q_vector(adapter, v_idx);
	}

	return -ENOMEM;
}

static int mnic_init_interrupt_scheme(struct mnic_adpter *adapter,bool msix)
{
	int ret;
	struct pci_dev *pdev = adpter->pdev;
	
	mnic_set_interrupt_capability(adapter,msix);
	
	ret = mnic_alloc_q_vectors(adapter);
	if(ret){
		pr_info("Unable to allocate memory for vectors\n");
		goto err_alloc_q_vectors;
	}

	//mnic_cache_ring_register(adpter);
	return 0;

err_alloc_q_vectors:
	igb_reset_interrupt_capability(adapter);
	return ret;
}

static void mnic_irq_disable(struct mnic_adapter *adapter)
{
	int i;
	for(i=0;i < adapter->num_q_vectors;i++){
		synchronize_irq(adapter->msix_entries[i].vector);
	}	
}

static int mnic_sw_init(struct mnic_adapter *adapter)
{
	uint32_t max_rss_queues;
	struct net_device *ndev = adapter->ndev;
	struct pci_device *pdev = adapter->pdev;

	adapter->tx_ring_count = MNIC_DEFAULT_TXD;
	adapter->rx_ring_count = MNIC_DEFAULT_RXD;

	adapter->tx_itr_setting = MNIC_DEFAULT_ITR;
	adapter->rx_itr_setting = MNIC_DEFAULT_ITR;

	adapter->tx_work_limit = MNIC_DEFAULT_TX_WORK;

	adapter->max_frame_size = ndev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
	adapter->min_fram_size  = ETH_ZLEN + ETH_FCS_LEN;

	adapter->rss_queues = min_t(uint32_t,max_rss_queues,num_online_cpu());

	spin_lock_init(&adapter->stat64_lock);

	max_rss_queues = MNIC_MAX_RX_QUEUES;
	adapter->rss_queues = min_t(uint32_t,max_rss_queues,num_online_cpus());

	if(mnic_init_interrupt_scheme(adapter,true)){
		pr_info("%s: Unable to allocate memory for queues \n");
		return -ENOMEM;
	}

	//やばいかも？
	//mnic_irq_disable(adpter);

	return 0;
}

static void mnic_probe(struct pci_dev *pdev,const struct pci_device_id *ent)
{
	int ret;
	int pci_using_dac;	
	void *bar0,*bar2,*bar4;
	uint64_t bar0_start,bar0_len;
	uint64_t bar2_start,bar2_len;
	uint64_t bar4_start,bar4_len;
	struct net_device *ndev;
	struct mnic_adpter *adapter;

	pr_info("%s: register nettlp modern nic device %s\n",
					__func__,pci_name(pdev));

	//see https://bootlin.com/doc/legacy/pci-drivers/pci-drivers.pdf 

	//wake up the device allocate I/O and memory regions of the device
	ret = pci_enable_device(pdev);
	if(ret){
		goto err1;
	}	

	//to read and write 64 bit memory
	ret = dma_set_mask_and_coherent(&pdev->dev,DMA_BIT_MASK(64));
	if(!ret){
		pci_using_dac = 1;	
	}
	else{
		ret = dma_set_mask_and_coherent(&pdev->dev,DMA_BIT_MASK(32));
		if(ret){
			pr_info("%s: No usable DMA configuration",__func__);
			goto err6;
		}
	}

	//reserver I/O region
	ret = pci_request_region(pdev,DRV_NAME);
	if(ret){
		goto err2;
	}

	//enable dma 
	pci_set_master(pdev);
	pci_save_state(pdev);

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
	ndev = allocate_etherdev(sizeof(*adapter));
	if(!ndev){
		goto err6;
	}
	//associate with the device private data
	SET_NETDEV_DEV(ndev,&pdev->dev);
	pci_set_drvdata(pdev,ndev);

	//access network device private data
	//like getting a pointer of net_device
	adapter = (struct nettlp_mnic_adpter *)netdev_priv(ndev);
	adapter->ndev = ndev;
	adapter->pdev = pdev;
	adapter->msg_enable = netif_msg_init(debug,DEFAULT_MSG_ENABLE);
	adapter->bar0 = bar0;
	adapter->bar2 = bar2;	
	adapter->bar4 = bar4;

	//decreare and device with more ore less than 32bit-bus master capability
	/*ret = pci_set_dma_mask(pdev,DMA_32BIT_MASK);
	if(!ret){
		ret = pci_set_cosistent_dma_mask(pdev,DMA_32BIT_MASK);
	}
	if(ret){
		pr_info("No suitable DMA available.\n");
		goto err6;
	}*/


	//get a virtual address of dma buffer and store physical address to variable 3
	/*adpter->txq = dma_alloc_coherent(&pdev->dev,sizeof(struct tx_queue)*TX_QUEUE_ENTRIES,&m_adpter->tx_desc_paddr,GFP_KERNEL);
	if(!m_adpter->txq){
		pr_info("%s: failed to alloc tx descriptor\n",__func__);
		goto err6;
	}	

	adpter->rxq = dma_alloc_coherent(&pdev->dev,sizeof(struct rx_queue)*RX_QUEUE_ENTRIES,&m_adpter->rx_desc_paddr,GFP_KERNEL);
	if(!m_adpter->rxq){
		pr_info("%s: failed to alloc rx descriptor\n",__func__);
		goto err6;
	}	

	adpter->tx_buf = dma_alloc_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,&adpter->tx_buf->paddr,GFP_KERNEL);
	if(!m_adpter->tx_buf){
		pr_info("%s: failed to alloc tx buffer\n",__func__);
		goto err6;
	}	

	adpter->rx_buf = dma_alloc_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,&adpter->rx_buf->paddr,GFP_KERNEL);
	if(!m_adpter->rx_buf){
		pr_info("%s: failed to alloc rx buffer\n",__func__);
		goto err6;
	}*/	

	spin_lock_init(&adapter->tx_lock);
	spin_lock_init(&adapter->rx_lock);

	mnic_get_mac(ndev->dev_addr,adapter->bar0->srcmac);	
	ndev->needs_free_netdev = true;
	ndev->netdev_ops = &nettlp_mnic_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_MAX_MTU;
	
	ret = register_netdev(ndev);
	if(ret){
		goto err7;
	}

	/*caffier off reporting is important to ethtool even BEFORE open*/
	netif_carrier_off(ndev);

	/*initialize tasklet for rx interrupt
	adpter->rx_tasklet = kmalloc(sizeof(struct tasklet_struct),GFP_KERNEL);
	if(m_adpter->rx_tasklet){
		ret = -ENOMEM;
		goto err7;
	}
	tasklet_init(m_adpter->rx_tasklet,rx_tasklet,(unsigned long)adpter);*/

	/* register interrupt
	ret = nettlp_register_interrupts(adpter);
	if(ret){
		goto err8;
	}*/

	//initialize the private structure
	ret = mnic_sw_init(adapter);
	if(ret){
		goto err9;		
	}

	nettlp_msg_init(bar4_start,PCI_DEVID(pdev->bus->number,pdev->devfn));

	//initialize and enable napi
	/*netif_napi_add(ndev,&m_adpter->napi,nettlp_mnic_poll,POOL_SIZE);
	napi_enable(&m_adpter->napi);
	m_adpter->napi_enabled = 1;*/

	//initialize each variable
	/*m_adpter->txq->tx_index = 0;
	m_adpter->txq->clean_index = 0;
	m_adpter->txq->num_entries = NUM_TX_QUEUE_ENTRIES;
	
	m_adpter->rxq->rx_index = 0;
	m_adpter->rxq->num_entries = NUM_RX_QUEUE_ENTRIES;
	
	m_adpter->tx_buf->index = 0;
	m_adpter->tx_buf->memp->index = 0;

	m_adpter->rx_buf->index = 0;
	m_adpter->rx_buf->memp->index = 0;

	m_adpter->bar4->tx->tdh = 0;
	m_adpter->bar4->tx->tdt = 0;
	m_adpter->bar4->rx->rdh = 0;
	m_adpter->bar4->rx->rdt = 0;*/

	//dev_pm_set_driver_flags(&pdev->dev,DPM_FLAG_NEVER_SKIP);
	//pm_runtime_put_noidle(&pdev->dev);

	pr_info("%s: probe finished.",__func__);
	
	return 0;

err9:
	mnic_clear_interrupt_scheme(adapter);
err8:
	kfree(m_adapter->rx_tasklet);
err7:
	unregister_netdev(dev);
/*err7:
	(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,(void*)m_adpter->txq,m_adpter->txq->tx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,(void*)m_adpter->rxq,m_adpter->rxq->rx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->tx_buf,&m_adpter->tx_buf->paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)m_adpter->rx_buf,&m_adpter->rx_buf->paddr);
*/
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


static void mnic_remove(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct mnic_adapter *adapter = netdev_priv(dev);

	pr_info("start remove pci config");

	kfree(adpter->rx_tasklet);

	nettlp_msg_fini();
	nettlp_unregister_interrupts(adapter);
	pci_free_irq_vectors(pdev);

	unregister_netdev(dev);

	/*dma_free_coherent(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,(void*)adpter->txq,adpter->txq->tx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,(void*)adpter->rxq,adpter->rxq->rx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)adpter->tx_buf,&adpter->tx_buf->paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)adpter->rx_buf,&adpter->rx_buf->paddr);*/

	iounmap(bar4);
	iounmap(bar2);
	iounmap(bar0);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	return;
}

struct pci_driver nettlp_mnic_pci_driver = {
	.name 		= DRV_NAME,
	.id_table	= mnic_pci_table,
	.probe		= mnic_probe,
	.remove		= mnic_remove,	
}

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
