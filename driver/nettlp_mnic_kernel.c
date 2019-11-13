#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/pci.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/ip_tunnels.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/netdevice.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/net_tstamp.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/pci-aspm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/sctp.h>
#include <linux/aer.h>
#include <linux/prefetch.h>
#include <linux/pm_runtime.h>

#include "nettlp_msg.h"
#include <nettlp_mnic.h>

#define NETTLP_MNIC_VERSION "0.0.1"
#define DRV_NAME 	    "nettlp_mnic_driver"
#define MNIC_DESC_RING_LEN  1

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)

static int debug = -1;
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
	uint32_t i = tx_ring->next_to_clean;
	struct mnic_tx_buffer *tx_buffer = &tx_ring->tx_buf_info[i];

	while (i != tx_ring->next_to_use) {
		struct mnic_adv_tx_descriptor *eop_desc, *tx_desc;

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
				tx_buffer = tx_ring->tx_buf_info;
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
			tx_buffer = tx_ring->tx_buf_info;
		}
	}

	//netdev_tx_reset_queue(txring_txq(tx_ring));

	/* reset next_to_use and next_to_clean */
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

static void mnic_clean_all_tx_rings(struct mnic_adapter *adapter)
{
	int i;
	
	for(i=0;i<adapter->num_tx_queues;i++){
		if(adapter->tx_ring[i]){
			mnic_clean_tx_ring(adapter->tx_ring[i]);
		}
	}
}


void mnic_free_tx_resource(struct mnic_ring *tx_ring)
{
	mnic_clean_tx_ring(tx_ring);
	
	vfree(tx_ring->tx_buf_info);
	tx_ring->tx_buf_info = NULL;
	
	if(tx_ring->desc){
		dma_free_coherent(tx_ring->dev,tx_ring->size,tx_ring->desc,tx_ring->dma);
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
	//struct mnic_adapter *adapter = netdev_priv(dev);
	
	//tx_ring->count = MNIC_DEFAULT_TXD;
	size = sizeof(struct mnic_tx_buffer)*tx_ring->count;
	
	tx_ring->tx_buf_info = vmalloc(size);
	if(!tx_ring->tx_buf_info){
		pr_info("%s: failed to alloc tx bufffer\n",__func__);
		goto err;
	}
	
	tx_ring->size = tx_ring->count*sizeof(struct mnic_adv_tx_descriptor);
	tx_ring->size = ALIGN(tx_ring->size,4096);
	
	tx_ring->desc = dma_alloc_coherent(dev,tx_ring->size,&tx_ring->dma,GFP_KERNEL);
	if(!tx_ring->desc){
		goto err;
	}
	
	/*notify descriptor base address*/
	//adapter->bar4->tx_desc_base = tx_ring->dma;
	//pr_info("notify tx descriptor base address -> %#llx\n",adapter->bar4->tx_desc_base);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	vfree(tx_ring->tx_buf_info);
	tx_ring->tx_buf_info = NULL;
	//dev_err("failed to allocate memory for tx descriptor ring\n");
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
				mnic_free_tx_resource(adapter->tx_ring[i]);
			}
			break;
		}
	}
	
	return ret;
}

static void mnic_clean_rx_ring(struct mnic_ring *rx_ring)
{
	uint32_t i = rx_ring->next_to_clean;
	dev_kfree_skb(rx_ring->skb);
	rx_ring->skb = NULL;

	/* Free all the Rx ring sk_buffs */
	while (i != rx_ring->next_to_alloc) {
		struct mnic_rx_buffer *buffer_info = &rx_ring->rx_buf_info[i];

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
				     MNIC_RX_DMA_ATTR);
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

static void mnic_clean_all_rx_rings(struct mnic_adapter *adapter)
{
	int i;
	
	for(i=0;i<adapter->num_rx_queues;i++){
		if(adapter->rx_ring[i]){
			mnic_clean_rx_ring(adapter->rx_ring[i]);
		}
	}
}
void mnic_free_rx_resources(struct mnic_ring *rx_ring)
{
	mnic_clean_rx_ring(rx_ring);

	vfree(rx_ring->rx_buf_info);
	rx_ring->rx_buf_info = NULL;

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
	//struct mnic_adapter *adapter = netdev_priv(dev);

	//rx_ring->count = MNIC_DEFAULT_RXD;

	size = sizeof(struct mnic_rx_buffer) * rx_ring->count;

	rx_ring->rx_buf_info = vmalloc(size);

	if (!rx_ring->rx_buf_info){
		goto err;
	}

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(struct mnic_adv_rx_descriptor);
	rx_ring->size = ALIGN(rx_ring->size, 4096);
	rx_ring->desc = dma_alloc_coherent(dev, rx_ring->size,
					   &rx_ring->dma, GFP_KERNEL);

	if (!rx_ring->desc){
		goto err;
	}

	/*notify rx descriptor address*/
	//adapter->bar4->rx_desc_base = rx_ring->dma;

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;

err:
	vfree(rx_ring->rx_buf_info);
	rx_ring->rx_buf_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");

	return -ENOMEM;
}

static int mnic_setup_all_rx_resources(struct mnic_adapter *adapter)
{
	int i,ret=0;
	
	for(i=0;i<adapter->num_rx_queues;i++){
		ret = mnic_setup_rx_resource(adapter->rx_ring[i]);
		if(ret){
			pr_info("%s: failed to set up rx resouce\n",__func__);
			for(i--;i>0;i--){
				mnic_free_rx_resources(adapter->rx_ring[i]);
			}
			break;
		}
	}

	return ret;
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
			sprintf(q_vector->name, "%s-TxRx-%u", ndev->name,
				q_vector->rx.ring->queue_idx);
		}
		else if (q_vector->tx.ring){
			sprintf(q_vector->name, "%s-tx-%u", ndev->name,
				q_vector->tx.ring->queue_idx);
		}
		else if (q_vector->rx.ring){
			sprintf(q_vector->name, "%s-rx-%u", ndev->name,
				q_vector->rx.ring->queue_idx);
		}
		else{
			sprintf(q_vector->name, "%s-unused", ndev->name);
		}

		ret = request_irq(adapter->msix_entries[vector].vector,mnic_msix_ring,0,q_vector->name,q_vector);
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
	//struct net_device *ndev = adapter->ndev;
	//struct pci_dev *pdev = adapter->pdev;
	
	ret = mnic_request_msix(adapter);

	if(ret==0){
		//うまくいけばここでおわり
		goto request_done;
	}
	else{
		pr_info("%s: koko ni kiteha dame!!!!!",__func__);
		pr_info("%s: outihe okeeri!!!",__func__);
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


static bool mnic_clean_tx_irq(struct mnic_q_vector *q_vector)
{
	//struct mnic_adapter *adapter = q_vector->adapter;
	struct mnic_ring *tx_ring = q_vector->tx.ring;
	struct mnic_tx_buffer *tx_buff;
	struct mnic_adv_tx_descriptor *tx_desc;
	//uint32_t total_bytes = 0
	//uint32_t total_packets = 0;
	uint32_t budget = q_vector->tx.work_limit;
	uint32_t i = tx_ring->next_to_clean;

	tx_buff = &tx_ring->tx_buf_info[i];
	tx_desc = MNIC_TX_DESC(tx_ring,i); 
	i -= tx_ring->count;

	do{
		struct mnic_adv_tx_descriptor *eop_desc = tx_buff->next_to_watch;
		if(!eop_desc){
			break;
		}

		read_barrier_depends();
	
		tx_buff->next_to_watch = NULL;

		//total_bytes += tx_buffer->bytecount;
		//total_packes += tx_buffer->gso_segs;
		
		dev_kfree_skb_any(tx_buff->skb);
		
		dma_unmap_single(tx_ring->dev,dma_unmap_addr(tx_buff,dma),dma_unmap_len(tx_buff,len),DMA_TO_DEVICE);
		/*clear all tx_buffer data*/
		tx_buff->skb = NULL;
		dma_unmap_len_set(tx_buff,len,0);
		
		/*clear last DMA location and unmap remaining buffers*/
		while(tx_desc != eop_desc){
			tx_buff++;
			tx_desc++;
			i++;
			if(unlikely(!i)){
				i -= tx_ring->count;
				tx_buff = tx_ring->tx_buf_info;
				tx_desc = MNIC_TX_DESC(tx_ring,0);
			}

			if(dma_unmap_len(tx_buff,len)){
				dma_unmap_page(tx_ring->dev,dma_unmap_addr(tx_buff,dma),dma_unmap_len(tx_buff,len),DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buff,len,0);
			}
		}

		/*move us one more past the eop_desc for prefeth*/
		tx_buff++;
		tx_desc++;
		i++;

		if(unlikely(!i)){
			i -= tx_ring->count;
			tx_buff = tx_ring->tx_buf_info;
			tx_desc = MNIC_TX_DESC(tx_ring,0);
		}

		/*issue prefetch for next tx descriptors*/
		prefetch(tx_desc);

		budget--;
	}while(likely(budget));

	i += tx_ring->count;
	tx_ring->next_to_clean = i;
	
	return !!budget;
}


static bool mnic_is_non_eop(struct mnic_ring *rx_ring,struct mnic_adv_rx_descriptor *rx_desc)
{
	u32 ntc = rx_ring->next_to_clean + 1;
	
	ntc = (ntc < rx_ring->count) ? ntc : 0;
	rx_ring->next_to_clean = ntc;
	
	prefetch(MNIC_RX_DESC(rx_ring,ntc));
	
	if(likely(mnic_test_staterr(rx_desc,MNIC_RXD_STAT_EOP))){
		return false;
	}
	
	return true;
}

static void mnic_pull_tail(struct mnic_ring *rx_ring,struct mnic_adv_rx_descriptor *rx_desc,struct sk_buff *skb)
{
	unsigned char *vaddr;
	unsigned int pull_len;
	struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[0];

	vaddr = skb_frag_address(frag);
	
	/*
	we need the header to contain the greater of either ETH_HLEN or 
	60 bytes if the skb->len is less than 60 for skb_pad
	*/
	pull_len = eth_get_headlen(vaddr,MNIC_RX_HDR_LEN);

	/*align pull length to size of long to optimize memcpy performance*/
	skb_copy_to_linear_data(skb,vaddr,ALIGN(pull_len,sizeof(long)));
	
	/*update all of the pointers*/
	skb_frag_size_sub(frag,pull_len);
	frag->page_offset += pull_len;
	skb->data_len -= pull_len;
	skb->tail += pull_len;
	
}
static bool mnic_cleanup_headers(struct mnic_ring *rx_ring,struct mnic_adv_rx_descriptor *rx_desc,struct sk_buff *skb)
{
	if(unlikely((mnic_test_staterr(rx_desc,MNIC_RXDEXT_ERR_FRAME_ERR_MASK)))){
		struct net_device *ndev = rx_ring->ndev;
		if(!(ndev->features & NETIF_F_RXALL)){
			dev_kfree_skb_any(skb);
			return true;
		}
	}

	/*place header in linear portion of buffer*/
	if(skb_is_nonlinear(skb)){
		mnic_pull_tail(rx_ring,rx_desc,skb);
	}

	if(unlikely(skb->len < 60)){
		int pad_len = 60 - skb->len;	
		
		if(skb_pad(skb,pad_len)){
			return true;
		}
		__skb_put(skb,pad_len);
	}
	
	return false;
}

static bool mnic_alloc_mapped_page(struct mnic_ring *rx_ring,struct mnic_rx_buffer *rb)
{
	struct page *page = rb->page;
	dma_addr_t dma;
	
	//finish this function if we already have page
	if(likely(page)){
		return true;
	}
	
	//pagesize 4096 = 0
	page = dev_alloc_pages(0);
	if(unlikely(!page)){
		pr_info("%s: failed to alloc page\n",__func__);
		return false;
	}
	
	dma = dma_map_page_attrs(rx_ring->dev,page,0,PAGE_SIZE,DMA_FROM_DEVICE,MNIC_RX_DMA_ATTR);

	if(dma_mapping_error(rx_ring->dev,dma)){
		__free_page(page);
		pr_info("%s: failed to map page\n",__func__);
		return false;
	}

	rb->dma = dma;
	rb->page = page;
	rb->page_offset = 0;
		
	return true;
}

void mnic_alloc_rx_buffers(struct mnic_ring *rx_ring,uint16_t cleaned_count)
{
	struct mnic_adv_rx_descriptor *rx_desc;
	struct mnic_rx_buffer *rb;
	uint16_t i = rx_ring->next_to_use;

	if(!cleaned_count){
		return;
	}

	rx_desc = MNIC_RX_DESC(rx_ring,i);
	rb = &rx_ring->rx_buf_info[i];
	i -= rx_ring->count;

	do{
		if(!mnic_alloc_mapped_page(rx_ring,rb)){
			break;
		}

		rx_desc->read.pkt_addr = cpu_to_le64(rb->dma + rb->page_offset);
		
		rx_desc++;
		rb++;
		i++;
	
		if(unlikely(!i)){
			rx_desc = MNIC_RX_DESC(rx_ring,0);
			rb = rx_ring->rx_buf_info;
			i -= rx_ring->count;
		}

		rx_desc->read.hdr_addr = 0;		
		cleaned_count--;	
	}while(cleaned_count);

	i += rx_ring->count;
	
	if(i != rx_ring->next_to_use){
		rx_ring->next_to_use = i;
		rx_ring->next_to_alloc = i;

		//set tail	
		//adapter->bar4->rx_desc->tail = i;
	}
}

/*static bool mnic_can_reuse_rx_page(struct mnic_rx_buffer *rx_buffer,struct page *page,unsigned int truesize)
{
	if(unlikely(page_to_nid(page) != numa_node_id())){
		return false;
	}

#if (PAGE_SIZE < 8192)
	//if we are only owner of page we can reuse it
	if(unlikely(page_count(page) != 1)){
		return false;
	}	
	
	//flip page offset to other buffer
	rx_buffer->page_offset ^= MNIC_RX_BUFSZ;

	atomic(&page->_count,2);
#else
	//move offset up to the next cache line
	rx_buffer->page_offset += truesize;
	if(rx_buffer->page_offset > (PAGE_SIZE - MNIC_RX_BUFSZ)){
		return false;
	}
	
	get_page(page);
#endif
	return true;
}*/

static bool mnic_can_reuse_rx_page(struct mnic_rx_buffer *rx_buffer)
{
	unsigned int pagecnt_bias = rx_buffer->pagecnt_bias;
	struct page *page = rx_buffer->page;
	
	/*if(unlikely(igb_page_is_reserved(page))return false;*/
	
	/*page size < 8192*/
	if(unlikely((page_ref_count(page)-pagecnt_bias)<1)){
		return false;
	}
	
	if(unlikely(!pagecnt_bias)){
		page_ref_add(page,USHRT_MAX);
		rx_buffer->pagecnt_bias = USHRT_MAX;
	}

	return true;
}

static bool mnic_add_rx_frag(struct mnic_ring *mnic_ring,struct mnic_rx_buffer *rx_buffer,struct mnic_adv_rx_descriptor *rx_desc,struct sk_buff *skb)
{
	struct page *page = rx_buffer->page;
	//uint32_t size = le16_to_cpu(rx_desc->wb.upper.length);
	uint32_t size = 2048; /*2048でほんとにいいのか問題*/
#if (PAGE_SIZE < 8192)
	unsigned int truesize = MNIC_RX_BUFSZ;
#else
	unsigned int truesize = ALIGN(size,L1_CACHE_BYTES);
#endif

	if((size <= MNIC_RX_HDR_LEN) && !skb_is_nonlinear(skb)){
		unsigned char *vaddr = page_address(page) + rx_buffer->page_offset;

		memcpy(__skb_put(skb,size),vaddr,ALIGN(size,sizeof(long)));

		if(likely(page_to_nid(page) == numa_node_id())){
			return true;
		}
	
		put_page(page);
		return false;
	}

	//for fragmentation
	skb_add_rx_frag(skb,skb_shinfo(skb)->nr_frags,page,rx_buffer->page_offset,size,truesize);

	return mnic_can_reuse_rx_page(rx_buffer/*,page,truesize*/);  
}

static struct sk_buff *mnic_fetch_rx_buffer(struct mnic_ring *rx_ring,struct mnic_adv_rx_descriptor *rx_desc,struct sk_buff *skb)
{
	struct page *page;
	struct mnic_rx_buffer *rx_buffer = &rx_ring->rx_buf_info[rx_ring->next_to_clean];
	
	page = rx_buffer->page;
	prefetchw(page);

	if(likely(!skb)){
		void *page_addr = page_address(page) + rx_buffer->page_offset;
		prefetch(page_addr);

		//allocate a skbuff for rx on a specific device and and align ip
		skb = netdev_alloc_skb_ip_align(rx_ring->ndev,MNIC_RX_HDR_LEN);
		if(unlikely(!skb)){
			return NULL;
		}
		
		prefetchw(skb->data);
	}
	
	/*we are reusing so sync this buffer for cpu use*/
	/*in order for the CPU and devices to see the most up-to-date and correct copy of the DMA buffer*/
	dma_sync_single_range_for_cpu(rx_ring->dev,rx_buffer->dma,rx_buffer->page_offset,MNIC_RX_BUFSZ,DMA_FROM_DEVICE);
	
	if(mnic_add_rx_frag(rx_ring,rx_buffer,rx_desc,skb)){
		mnic_can_reuse_rx_page(/*rx_ring,page,*/rx_buffer);
	}
	else{
		dma_unmap_page(rx_ring->dev,rx_buffer->dma,PAGE_SIZE,DMA_FROM_DEVICE);
	}

	rx_buffer->page = NULL;
	
	return skb;
}

static bool mnic_clean_rx_irq(struct mnic_q_vector *q_vector,const int budget)
{
	struct mnic_ring *rx_ring = q_vector->rx.ring;
	struct sk_buff *skb = rx_ring->skb;
	uint32_t total_bytes = 0,total_packets = 0;
	uint16_t cleaned_count = mnic_desc_unused(rx_ring); 

	while(likely(total_packets < budget)){
		struct mnic_adv_rx_descriptor *rx_desc;

		if(cleaned_count >= MNIC_RX_BUFFER_WRITE){
			mnic_alloc_rx_buffers(rx_ring,cleaned_count);
			cleaned_count = 0;			
		}

		//dma_rmb();

		rx_desc = MNIC_RX_DESC(rx_ring,rx_ring->next_to_clean);
		
		skb = mnic_fetch_rx_buffer(rx_ring,rx_desc,skb);
		if(!skb){
			break;
		}
		
		cleaned_count++;
		
		if(mnic_is_non_eop(rx_ring,rx_desc)){
			continue;
		}
		if(mnic_cleanup_headers(rx_ring,rx_desc,skb)){
			skb = NULL;
			continue;
		}
		
		total_bytes += skb->len;
		//mnic_process_skb_fields(rx_ring,rx_desc,skb);
	
		skb->protocol = eth_type_trans(skb,rx_ring->ndev);
		skb->ip_summed = CHECKSUM_NONE;

		napi_gro_receive(&q_vector->napi,skb);

		skb = NULL;
		
		total_packets++;
	}
	
	rx_ring->skb = skb;
	rx_ring->rx_stats.packets += total_packets;
	rx_ring->rx_stats.bytes += total_bytes;
	
	q_vector->rx.total_packets += total_packets;
	q_vector->rx.total_bytes += total_bytes;

	if(cleaned_count){
		mnic_alloc_rx_buffers(rx_ring,cleaned_count);
	}
	
	return (total_packets < budget);
}	

static int mnic_poll(struct napi_struct *napi,int budget)
{
	bool clean_complete = true;
	int work_done = 0;
	struct mnic_q_vector *q_vector = container_of(napi,struct mnic_q_vector,napi);

#ifdef CONFIG_MNIC_DCA
	mnic_update_dca(q_vector);
#endif

	if(q_vector->tx.ring){
		clean_complete = mnic_clean_tx_irq(q_vector);
	}
	if(q_vector->rx.ring){
		int cleaned = mnic_clean_rx_irq(q_vector,budget);
		work_done += cleaned;
		if(cleaned >= budget){
			clean_complete = false;
		}
	}

	if(!clean_complete){
		return budget;
	}
	
	if(likely(napi_complete_done(napi,work_done))){
		//rmnic_ring_irq_enable(q_vector);
		pr_info("%s:napi complete done",__func__);
	}

	return min(work_done,budget-1);
}

static void mnic_free_irq(struct mnic_adapter *adapter)
{
	int i;
	int vector = 0;
	free_irq(adapter->msix_entries[vector++].vector,adapter);

	for(i=0;i<adapter->num_q_vectors;i++){
		free_irq(adapter->msix_entries[vector++].vector,adapter->q_vector[i]);
	}
}

static void mnic_free_all_rx_resources(struct mnic_adapter *adapter)
{
	int i;
	
	for(i=0;i<adapter->num_rx_queues;i++){
		if(adapter->rx_ring[i]){
			mnic_free_rx_resources(adapter->rx_ring[i]);
		}
	}
}
static int __mnic_open(struct net_device *ndev,bool resuming)
{
	int ret,i;
	struct mnic_adapter *adapter = netdev_priv(ndev);
	//struct pci_dev *pdev = adapter->pdev;

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
	for(i=0;i<adapter->num_rx_queues;i++){
		struct mnic_ring *rx_ring = adapter->rx_ring[i];
		mnic_alloc_rx_buffers(rx_ring,mnic_desc_unused(rx_ring));
	}
	
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
	mnic_free_all_tx_resources(adapter);

err_setup_tx:
	//igb_reset(adapter);
	pr_info("%s:igb_reset\n",__func__);

	return ret;
}

static int mnic_open(struct net_device *ndev)
{
	return __mnic_open(ndev,false);
}

void mnic_unmap_and_free_tx_resource(struct mnic_ring *ring,
				    struct mnic_tx_buffer *tx_buffer)
{
	if (tx_buffer->skb) {
		dev_kfree_skb_any(tx_buffer->skb);
		if (dma_unmap_len(tx_buffer, len))
			dma_unmap_single(ring->dev,
					 dma_unmap_addr(tx_buffer, dma),
					 dma_unmap_len(tx_buffer, len),
					 DMA_TO_DEVICE);
	} else if (dma_unmap_len(tx_buffer, len)) {
		dma_unmap_page(ring->dev,
			       dma_unmap_addr(tx_buffer, dma),
			       dma_unmap_len(tx_buffer, len),
			       DMA_TO_DEVICE);
	}
	tx_buffer->next_to_watch = NULL;
	tx_buffer->skb = NULL;
	dma_unmap_len_set(tx_buffer, len, 0);
	/* buffer_info must be completely set up in the transmit path */
}

static int mnic_tx_map(struct mnic_ring *tx_ring,struct mnic_tx_buffer *first,const uint8_t hdr_len,struct mnic_adapter *adapter)
{
	struct sk_buff *skb = first->skb;
	struct mnic_tx_buffer *tx_buff;
	struct mnic_adv_tx_descriptor *tx_desc;
	skb_frag_t *frag;
	dma_addr_t dma;
	unsigned int data_len,size;
	//uint32_t tx_flags = first->tx_flags;
	uint16_t i = tx_ring->next_to_use;
	
	tx_desc = MNIC_TX_DESC(tx_ring,i);
	size = skb_headlen(skb);
	data_len = skb->data_len;

	dma = dma_map_single(tx_ring->dev,skb->data,size,DMA_TO_DEVICE);
	tx_buff = first;

	for(frag = &skb_shinfo(skb)->frags[0];;frag++){
		if(dma_mapping_error(tx_ring->dev,dma)){
			goto dma_error;
		}
		
		dma_unmap_len_set(tx_buff,len,size);
		dma_unmap_addr_set(tx_buff,dma,dma);
	
		tx_desc->read.buffer_addr = cpu_to_le64(dma);
		
		while(unlikely(size > MNIC_MAX_DATA_PER_TXD)){
			//tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type ^ MNIC_MAX_DATA_PER_TXD);
			i++;
			tx_desc++;
	
			if(i == tx_ring->count){
				tx_desc = MNIC_TX_DESC(tx_ring,0);
				i = 0;
			}
			
			tx_desc->read.olinfo_status = 0;
			
			dma += MNIC_MAX_DATA_PER_TXD;
			size -= MNIC_MAX_DATA_PER_TXD;

			tx_desc->read.buffer_addr = cpu_to_le64(dma);
		}
	
		if(likely(!data_len)){
			break;
		}

		//tx_desc->read.cmd_type_len = cpu_to_le64(cmd_type ^ size);

		i++;
		tx_desc++;
		if(i == tx_ring->count){
			tx_desc = MNIC_TX_DESC(tx_ring,0);
			i=0;
		}
		tx_desc->read.olinfo_status = 0;
		
		size = skb_frag_size(frag);
		data_len -= size;
		
		dma = skb_frag_dma_map(tx_ring->dev,frag,0,size,DMA_TO_DEVICE);
		tx_buff = &tx_ring->tx_buf_info[i];
	}

	//netdev_tx_sent_queue(txring_txq(tx_ring),first->bytecount);

	dma_wmb();

	first->next_to_watch = tx_desc;

	i++;
	if(tx_ring->count == i){
		i=0;
	}
	
	tx_ring->next_to_use = i;
	//bar4の構造体のindexにtx_tailをとうろく
	adapter->bar4->tx_desc_tail = i;
	
	//mnic_maybe_stop_tx(tx_ring,DESC_NEEDED);
	//mmiowb();

	return 0;

dma_error:
	dev_err(tx_ring->dev, "TX DMA map failed\n");

	/* clear dma mappings for failed tx_buffer_info map */
	for (;;) {
		tx_buff = &tx_ring->tx_buf_info[i];
		mnic_unmap_and_free_tx_resource(tx_ring, tx_buff);
		if (tx_buff == first)
			break;
		if (i == 0)
			i = tx_ring->count;
		i--;
	}

	tx_ring->next_to_use = i;
	return -1;
}

static int __mnic_close(struct net_device *ndev,bool suspending)
{
	int i;
	struct mnic_adapter *adapter = netdev_priv(ndev);
	struct pci_dev *pdev = adapter->pdev;

	if(!suspending){
		pm_runtime_get_sync(&pdev->dev);
	}
	
	//mnic_down
	
	netif_carrier_off(ndev);
	netif_tx_stop_all_queues(ndev);
	
	for(i=0;i<adapter->num_q_vectors;i++){
		if(adapter->q_vector[i]){
			napi_synchronize(&adapter->q_vector[i]->napi);
			napi_disable(&adapter->q_vector[i]->napi);
		}
	}
	
	//mnic_clean_all_tx_ring(adapter);
	//mnic_clean_all_rx_ring(adapter);

	mnic_free_irq(adapter);
	
	mnic_free_all_tx_resources(adapter);
	mnic_free_all_rx_resources(adapter);
	
	if(!suspending){
		pm_runtime_get_sync(&pdev->dev);
	}

	return 0;
}
int mnic_close(struct net_device *ndev)
{
	if(netif_device_present(ndev) || ndev->dismantle){
		return __mnic_close(ndev,false);
	}
	return 0;
}

static netdev_tx_t mnic_xmit_frame_ring(struct sk_buff *skb,struct mnic_ring *tx_ring,struct mnic_adapter *adapter)
{
	struct mnic_tx_buffer *tx_buff;
	uint8_t hdr_len = 0;

	pr_info("%s\n",__func__);
	//uint32_t flags;
	//uint32_t count = TXD_USE_COUNT(skb_headlen(skb));

	tx_buff = &tx_ring->tx_buf_info[tx_ring->next_to_use];
	tx_buff->skb = skb;
	tx_buff->bytecount = skb->len;	
	//tx_buff->gso_segs = 1;
	//tx_buff->tx_flags = tx_flags;
	//tx_buff->protocol = protocol;
	
	mnic_tx_map(tx_ring,tx_buff,hdr_len,adapter);
	
	//mnic_maybe_stop_tx(tx_ring,DESC_NEEDED);

	return NETDEV_TX_OK;
}

static inline struct mnic_ring *mnic_tx_queue_mapping(struct mnic_adapter *adapter,struct sk_buff *skb)
{
	uint32_t r_idx = skb->queue_mapping;
	
	if(r_idx >= adapter->num_tx_queues){
		r_idx = r_idx % adapter->num_tx_queues;
	}

	pr_info("%s: queue mapping is %d\n",__func__,r_idx);
	return adapter->tx_ring[r_idx];
}

static netdev_tx_t nettlp_mnic_xmit_frame(struct sk_buff *skb,struct net_device *netdev)
{
	struct mnic_adapter *adapter = netdev_priv(netdev);
	
	if(skb_put_padto(skb,17)){
		return NETDEV_TX_OK;
	}

	return mnic_xmit_frame_ring(skb,mnic_tx_queue_mapping(adapter,skb),adapter);
}

static int nettlp_mnic_set_mac(struct net_device *ndev,void *p)
{
	struct mnic_adapter *adapter = netdev_priv(ndev);	
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
	.ndo_open		= mnic_open, 
	.ndo_stop		= mnic_close,
	.ndo_start_xmit 	= nettlp_mnic_xmit_frame,
	.ndo_get_stats64  	= ip_tunnel_get_stats64,
	.ndo_change_mtu 	= eth_change_mtu,
	.ndo_validate_addr 	= eth_validate_addr,
	.ndo_set_mac_address	= nettlp_mnic_set_mac,
};

/*void rx_tasklet(unsigned long tasklet_data)
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
}*/

//NAPI Rx polling callback
//@napi: napi polling structure
//budget: count of how many packets we should handle
/*static int nettlp_mnic_poll(struct napi_structure *napi,int budget)
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
*/

//use MSI-X
/*static int nettlp_mnic_interrupts(struct nettlp_mnic_adpter *m_adpter)
{
G_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)

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

/*static void nettlp_unregister_interrupts(struct mnic_adapter *adapter)
{
	free_irq(pci_irq_vector(m_adapter->pdev,0),adapter);
	free_irq(pci_irq_vector(m_adapter->pdev,1),m_adapter);
} */

static const struct pci_device_id mnic_pci_tbl[] = {
	{0x3776,0x8022,PCI_ANY_ID,PCI_ANY_ID,0,0,0},
	{0,}
};
MODULE_DEVICE_TABLE(pci,mnic_pci_tbl);

static void mnic_reset_q_vector(struct mnic_adapter *adapter, int v_idx)
{
	struct mnic_q_vector *q_vector = adapter->q_vector[v_idx];

	if(!q_vector){
		return;
	}

	if (q_vector->tx.ring){
		adapter->tx_ring[q_vector->tx.ring->queue_idx] = NULL;
	}

	if (q_vector->rx.ring){
		adapter->tx_ring[q_vector->rx.ring->queue_idx] = NULL;
	}

	netif_napi_del(&q_vector->napi);
}

static void mnic_reset_interrupt_capability(struct mnic_adapter *adapter)
{
	/*pci_disable_msix(adapter->pdev);
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;*/

	int v_idx = adapter->num_q_vectors;
	pci_disable_msix(adapter->pdev);
	while(v_idx--){
		mnic_reset_q_vector(adapter,v_idx);
	}
}

static void mnic_free_q_vector(struct mnic_adapter *adapter, int v_idx)
{
	struct mnic_q_vector *q_vector = adapter->q_vector[v_idx];

	adapter->q_vector[v_idx] = NULL;
	if(q_vector){
		kfree_rcu(q_vector, rcu);
	}
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

static void mnic_set_interrupt_capability(struct mnic_adapter *adapter,bool msix)
{
	int ret;
	int numvecs,i;

	//number of q for tx and rx(currently 8 or num_cpu_core,but maybe 16 good)
	if(adapter->rss_queues > MAX_MSIX_ENTRIES){
		pr_info("%s: adpter->rss_queus over MAX MSIX ENTRIES",__func__);
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
	//adapter->msix_entries = kcalloc(numvecs,sizeof(struct msix_entry),GFP_KERNEL);
	
	for(i=0;i<numvecs;i++){
		adapter->msix_entries[i].entry = i;	
	}
	
	ret = pci_enable_msix_range(adapter->pdev,adapter->msix_entries,numvecs,numvecs);

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

static int mnic_alloc_q_vector(struct mnic_adapter *adapter,int v_count,int v_idx,int txr_count,int txr_idx,int rxr_count,int rxr_idx)
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
		ring->queue_idx = txr_idx;

		//u64_stats_init(&ring->tx_syncp);
		//u64_stats_init(&ring->tx_syncp2);

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
	
		ring->count = adapter->rx_ring_count;
		ring->queue_idx = rxr_idx;
	
		//u64_stats_init(&ring->rx_sync);

		adapter->rx_ring[rxr_idx] = ring;
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
	
	if(q_vectors >= (rxr_remaining + txr_remaining)){
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
		ret = mnic_alloc_q_vector(adapter,q_vectors,v_idx,tqpv,txr_idx,rqpv,rxr_idx);
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
		mnic_free_q_vector(adapter, v_idx);
	}

	return -ENOMEM;
}

static int mnic_init_interrupt_scheme(struct mnic_adapter *adapter,bool msix)
{
	int ret;
	//struct pci_dev *pdev = adapter->pdev;
	
	mnic_set_interrupt_capability(adapter,msix);
	
	ret = mnic_alloc_q_vectors(adapter);
	if(ret){
		pr_info("Unable to allocate memory for vectors\n");
		goto err_alloc_q_vectors;
	}

	//mnic_cache_ring_register(adpter);
	return 0;

err_alloc_q_vectors:
	mnic_reset_interrupt_capability(adapter);
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
	//struct pci_device *pdev = adapter->pdev;

	adapter->tx_ring_count = MNIC_DEFAULT_TXD;
	adapter->rx_ring_count = MNIC_DEFAULT_RXD;

	adapter->tx_itr_setting = MNIC_DEFAULT_ITR;
	adapter->rx_itr_setting = MNIC_DEFAULT_ITR;

	adapter->tx_work_limit = MNIC_DEFAULT_TX_WORK;

	adapter->max_frame_size = ndev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
	adapter->min_frame_size  = ETH_ZLEN + ETH_FCS_LEN;

	spin_lock_init(&adapter->stats64_lock);

	max_rss_queues = MNIC_MAX_RX_QUEUES;
	adapter->rss_queues = min_t(uint32_t,max_rss_queues,num_online_cpus());

	if(mnic_init_interrupt_scheme(adapter,true)){
		pr_info("%s: Unable to allocate memory for queues \n",__func__);
		return -ENOMEM;
	}

	//やばいかも？
	mnic_irq_disable(adapter);

	return 0;
}

static int mnic_probe(struct pci_dev *pdev,const struct pci_device_id *ent)
{
	int ret;
	int pci_using_dac;	
	void *bar0,*bar2,*bar4;
	uint64_t bar0_start,bar0_len;
	uint64_t bar2_start,bar2_len;
	uint64_t bar4_start,bar4_len;
	struct net_device *ndev;
	struct mnic_adapter *adapter;

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
	ret = pci_request_regions(pdev,DRV_NAME);
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
	ndev = alloc_etherdev_mq(sizeof(struct mnic_adapter),MNIC_MAX_TX_QUEUES);
	if(!ndev){
		goto err6;
	}

	//associate with the device private data
	SET_NETDEV_DEV(ndev,&pdev->dev);
	pci_set_drvdata(pdev,ndev);

	//access network device private data
	//like getting a pointer of net_device
	adapter = (struct mnic_adapter *)netdev_priv(ndev);
	adapter->ndev = ndev;
	adapter->pdev = pdev;
	adapter->msg_enable = netif_msg_init(debug,DEFAULT_MSG_ENABLE);
	adapter->bar0 = bar0;
	adapter->bar2 = bar2;	
	adapter->bar4 = bar4;
	adapter->bar4->tx_desc_tail = 0;
	adapter->bar4->rx_desc_tail = 0;
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

	nettlp_msg_init(bar4_start,PCI_DEVID(pdev->bus->number,pdev->devfn),bar2);

	//dev_pm_set_driver_flags(&pdev->dev,DPM_FLAG_NEVER_SKIP);
	//pm_runtime_put_noidle(&pdev->dev);

	pr_info("%s: probe finished.",__func__);
	
	return 0;

err9:
	mnic_clear_interrupt_scheme(adapter);
//err8:
	//kfree(m_adapter->rx_tasklet);
err7:
	unregister_netdev(ndev);
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

	//kfree(adpter->rx_tasklet);

	nettlp_msg_fini();
	//nettlp_unregister_interrupts(adapter);
	//pci_free_irq_vectors(pdev);

	unregister_netdev(dev);

	/*dma_free_coherent(&pdev->dev,sizeof(struct tx_queue)*MNIC_TX_QUEUE_ENTRIES,(void*)adpter->txq,adpter->txq->tx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct rx_queue)*MNIC_RX_QUEUE_ENTRIES,(void*)adpter->rxq,adpter->rxq->rx_desc_paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)adpter->tx_buf,&adpter->tx_buf->paddr);

	dma_free_coherent(&pdev->dev,sizeof(struct packet_buffer)*PKT_BUF_ENTRY_SIZE,(void*)adpter->rx_buf,&adpter->rx_buf->paddr);*/

	iounmap(adapter->bar4);
	iounmap(adapter->bar2);
	iounmap(adapter->bar0);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	return;
}

struct pci_driver nettlp_mnic_pci_driver = {
	.name 		= DRV_NAME,
	.id_table	= mnic_pci_tbl,
	.probe		= mnic_probe,
	.remove		= mnic_remove,	
};

static int __init nettlp_mnic_module_init(void)
{
	pr_info("nettlp_mnic is loaded\n");

	return pci_register_driver(&nettlp_mnic_pci_driver);
}
module_init(nettlp_mnic_module_init);

static void __exit nettlp_mnic_module_exit(void)
{
	pci_unregister_driver(&nettlp_mnic_pci_driver);

	pr_info("nettlp_mnic is unloaded\n");

	return;
}
module_exit(nettlp_mnic_module_exit);

MODULE_AUTHOR("Ryusei Shiiba <siiba@sfc.wide.ad.jp>");
MODULE_DESCRIPTION("nettlp_mnic_kernel");
MODULE_LICENSE("GPL");
MODULE_VERSION(NETTLP_MNIC_VERSION);
