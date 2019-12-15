/*TX/RX descriptor defines*/
#define MNIC_DEFAULT_TXD	256
#define MNIC_MIN_TXD		80
#define MNIC_DEFAULT_TX_WORK    128
#define MNIC_MAX_TXD		4096
#define MNIC_MAX_TXD_PWR	15
#define MNIC_MAX_DATA_PER_TXD   (1u << MNIC_MAX_TXD_PWR)

#define MNIC_DEFAULT_RXD	256
#define MNIC_MIN_RXD		80
#define MNIC_MAX_RXD		4096

#define MAX_Q_VECTORS		8

#define MNIC_MAX_TX_QUEUES	8
#define MNIC_MAX_RX_QUEUES	8
#define MAX_MSIX_ENTRIES	16

#define MNIC_DEFAULT_ITR	3

#define MNIC_RX_BUFFER_WRITE	16

#define MNIC_RX_HDR_LEN		256
#define MNIC_RX_BUFSZ		2048

#define MNIC_RXD_STAT_EOP	0x02

#define DESC_NEEDED		(MAX_SKB_FRAGS + 4)

#define MNIC_RXDEXT_STATERR_LB    0x00040000
#define MNIC_RXDEXT_STATERR_CE    0x01000000
#define MNIC_RXDEXT_STATERR_SE    0x02000000
#define MNIC_RXDEXT_STATERR_SEQ   0x04000000
#define MNIC_RXDEXT_STATERR_CXE   0x10000000
#define MNIC_RXDEXT_STATERR_TCPE  0x20000000
#define MNIC_RXDEXT_STATERR_IPE   0x40000000
#define MNIC_RXDEXT_STATERR_RXE   0x80000000

/* Same mask, but for extended and packet split descriptors */
#define MNIC_RXDEXT_ERR_FRAME_ERR_MASK ( \
    MNIC_RXDEXT_STATERR_CE  |            \
    MNIC_RXDEXT_STATERR_SE  |            \
    MNIC_RXDEXT_STATERR_SEQ |            \
    MNIC_RXDEXT_STATERR_CXE |            \
    MNIC_RXDEXT_STATERR_RXE)

/*#define IGB_RX_PTHRESH	((hw->mac.type == e1000_i354) ? 12 : 8)
#define IGB_RX_HTHRESH	8
#define IGB_TX_PTHRESH	((hw->mac.type == e1000_i354) ? 20 : 8)
#define IGB_TX_HTHRESH	1
#define IGB_RX_WTHRESH	((hw->mac.type == e1000_82576 && \
			  (adapter->flags & IGB_FLAG_HAS_MSIX)) ? 1 : 4)
#define IGB_TX_WTHRESH	((hw->mac.type == e1000_82576 && \
			  (adapter->flags & IGB_FLAG_HAS_MSIX)) ? 1 : 16)
*/

#define MNIC_RX_DMA_ATTR (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING)

struct mnic_adapter adapter;

struct descriptor{
	uint64_t addr;
	uint64_t length;
}__attribute__((packed));

/*struct mnic_adv_tx_descriptor{
	struct {
		__le64 buffer_addr;//address of descriptor's data buf
		__le64 cmd_type_len;
		__le32 olinfo_status;
	}read;
	struct {
		__le64 rsvd;	
		__le32 nxtseq_seed;
		__le32 status;
	}wb;
};*/

/*struct mnic_adv_rx_descriptor{
	struct {
		__le64 pkt_addr; //Packet buffer address 
		__le64 hdr_addr; // Header buffer address 
	} read;
	struct {
		struct {
			union {
				__le32 data;
				struct {
					__le16 pkt_info; // RSS, Pkt type 
					__le16 hdr_info; // Splithdr, hdrlen 
				} hs_rss;
			} lo_dword;
			union {
				__le32 rss; // RSS Hash 
				struct {
					__le16 ip_id; // IP id 
					__le16 csum; // Packet Checksum 
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error; // ext status/error 
			__le16 length; // Packet length 
			__le16 vlan; // VLAN tag 
		} upper;
	} wb;  // writeback 
};*/

struct mnic_rx_register{
	uint64_t rdba;//receive base address register rdbl(32bit) rdbh(32bit) (low&high)
	uint64_t rdlen;//receive descriptor length 
	uint64_t rdh;//receive descriptor head
	uint64_t rdt;//receive descriptor tail
}__attribute__((packed));

struct mnic_tx_register{
	uint64_t tdba;//transmit descriptor base address tdbl(32bit) tdbh(32bit) (low&high)
	uint64_t tdlen;//transmit descirptor length
	uint64_t tdh;//transmit descriptor head
	uint64_t tdt;//transmit descriptor tail
}__attribute__((packed));

struct mnic_bar4
{	
	uint64_t tx_desc_base[8];
	uint64_t rx_desc_base[8];
	
	uint64_t tx_desc_tail;
	uint64_t rx_desc_tail;

	//uint32_t enabled;
}__attribute__((packed));

struct mnic_bar0{
	uint32_t magic;
	
	uint8_t dstmac[6];
	uint16_t rsv1;

	uint8_t srcmac[6];
	uint16_t rsv2;

	__be32 srcip;
	__be32 dstip;
};

struct memory_pool{
	void *base_addr;
	uint32_t base;
	uint32_t mem_size;
	uint32_t mem_idx;
	uint32_t free_area_top;
	uint32_t free_area[];
};

/*struct packet_buffer{
	struct memory_pool *memp;
	dma_addr_t paddr;
	uint32_t index;
	uint32_t size;
	uint8_t data[] __attribute_((aligned(64));
};*/

struct tx_queue{
	volatile union nettlp_mnic_tx_desc *nmtd;
	uint16_t num_entries;
	uint16_t tx_index;
	uint16_t clean_index;	
	//void *virtual_addr[];
	//struct packet_buffer *tx_pool;//buffer pool for packet??
	//struct packet_buffer *pkt_addr_backup;//save a copy of packet buffer address for writeback descriptor
};

struct rx_queue{
	volatile union nettlp_mnic_rx_desc *nmrd;
	uint16_t num_entries;
	uint16_t rx_index;
	//dma_addr_t rx_desc_paddr;
	//uint16_t *virtual_addr[];
	//struct packet_buffer *rx_pool;// buffer pool for packet??
	//struct packet_buffer *pkt_addr_backup;//save a copy of packet buffer address for writeback descriptor
};

/*struct nettlp_mnic{
	//tx descriptor(max queue 4096,min queue 512)
	struct tx_queue *txq;
	//rx descriptor(max queue 4096,min queue 512)
	struct rx_queue *rxq;
	struct mnic_bar4 mbar4;
};*/

struct mnic_tx_buffer{
	struct descriptor *next_to_watch;
	uint16_t time_stamp;
	struct sk_buff *skb;
	uint8_t bytecount;
	DEFINE_DMA_UNMAP_ADDR(dma);
	DEFINE_DMA_UNMAP_LEN(len);
	uint32_t tx_flags;
};

struct mnic_rx_buffer{
	struct descriptor *next_to_watch;
	dma_addr_t dma;
	struct page *page;

#if (BITS_PER_LONG > 32) || (PAGE_SIZE >= 65536)
	uint32_t page_offset;
#else
	uint16_t page_offset;
#endif
	uint16_t pagecnt_bias;
};

struct mnic_tx_q_stats{
	uint64_t packets;
	uint64_t bytes;	
	uint64_t restart_queue;
	uint64_t restart_queue2;
};

struct mnic_rx_q_stats{
	uint64_t packets;
	uint64_t bytes;	
	uint64_t drops;
	uint64_t csum_err;
	uint64_t alloc_failed;
};

struct mnic_ring_container{
	struct mnic_ring *ring;
	uint8_t total_bytes;
	uint8_t total_packets;
	uint16_t work_limit;
	uint8_t count;
	uint8_t itr;
};

struct mnic_ring{
	struct mnic_q_vector *q_vector;

	struct net_device *ndev;
	struct device *dev;
	union{
		struct mnic_tx_buffer *tx_buf_info;
		struct mnic_rx_buffer *rx_buf_info;
	};
	void *desc;
	unsigned long flags;
	void __iomem *tail;
	dma_addr_t dma;
	uint64_t size;
	uint16_t count;
	uint8_t queue_idx; //logical index 
	uint8_t reg_idx; //physical index
	
	uint16_t next_to_clean;
	uint16_t next_to_use;
	uint16_t next_to_alloc;

	union{
		struct{
			struct mnic_tx_q_stats tx_stats;
			struct u64_stats_sync tx_syncp;
			struct u64_stats_sync tx_syncp2;
		};
		struct{
			struct sk_buff *skb;
			struct mnic_rx_q_stats rx_stats;
			struct u64_stats_sync rx_syncp;
		};
		
	};
} ____cacheline_internodealigned_in_smp;

struct mnic_q_vector{
	struct mnic_adapter *adapter;
	int cpu;
	uint32_t eims_value;

	uint16_t itr_val;
	uint8_t set_itr;
	void __iomem *itr_register;

	struct mnic_ring_container rx,tx;
	
	struct napi_struct napi;
	struct rcu_head rcu;
	char name[IFNAMSIZ + 9];
	
	struct mnic_ring ring[0] ____cacheline_internodealigned_in_smp;
};

struct mnic_adapter{
	struct pci_dev *pdev;
	struct net_device *ndev;
	
	struct mnic_bar0 *bar0;
	struct mnic_bar4 *bar4;
	uint64_t bar4_start;
	void *bar2;

	uint16_t state;
	uint8_t flags;
	
	uint8_t num_q_vectors;
	struct msix_entry msix_entries[MAX_MSIX_ENTRIES];

	uint32_t tx_itr_setting;
	uint32_t rx_itr_setting;
	uint16_t tx_itr;
	uint16_t rx_itr;

	uint16_t tx_work_limit;
	uint32_t tx_timeout_count;
	uint8_t num_tx_queues;
	struct mnic_ring *tx_ring[16];
	
	uint8_t num_rx_queues;
	struct mnic_ring *rx_ring[16];

	uint32_t max_frame_size;
	uint32_t min_frame_size;

	uint8_t __iomem *io_addr;

	struct mnic_q_vector *q_vector[MAX_Q_VECTORS];
	
	uint16_t tx_ring_count;
	uint16_t rx_ring_count;

	spinlock_t tx_lock;
	spinlock_t rx_lock;
	spinlock_t stats64_lock;

	struct tasklet_struct *rx_tasklet;
	struct napi_struct napi;

	uint8_t msg_enable;
	uint8_t napi_enabled;
	uint32_t rss_queues;
	//struct tx_queue *txq;
	//struct rx_queue *rxq;

	//struct pkt_buffer *tx_buf;
	//struct pkt_buffer *rx_buf;
#define TX_STATE_READY 1
#define RX_STATE_BUSY  2

	//uint32_t tx_state;	
	//uint8_t  napi_enabled;
};

static inline int mnic_desc_unused(struct mnic_ring *ring)
{
	if (ring->next_to_clean > ring->next_to_use)
		return ring->next_to_clean - ring->next_to_use - 1;

	return ring->count + ring->next_to_clean - ring->next_to_use - 1;
}

static inline __le32 mnic_test_staterr(struct descriptor *rx_desc,const u32 stat_err_bits)
{
	//return rx_desc->wb.upper.status_error & cpu_to_le32(stat_err_bits);
	return 0;
}

#define MNIC_TX_DESC(R,i)	\
	(&(((struct descriptor *)((R)->desc))[i]))

#define MNIC_RX_DESC(R,i)	\
	(&(((struct descriptor *)((R)->desc))[i]))

#define mnic_get_mac(dst, src) do {			\
		dst[0] = src[5];		\
		dst[1] = src[4];		\
		dst[2] = src[3];		\
		dst[3] = src[2];		\
		dst[4] = src[1];		\
		dst[5] = src[0];		\
	}while(0)

#ifndef NDEBUG
#define debug(fmt, ...) do {\
	fprintf(stderr, "[DEBUG] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)
#else
#define debug(fmt, ...) do {} while(0)
#undef assert
#define assert(expr) (void) (expr)
#endif

#define info(fmt, ...) do {\
	fprintf(stdout, "[INFO ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)



