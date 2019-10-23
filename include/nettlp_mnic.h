union mnic_tx_descriptor{
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
};

union mnic_rx_descriptor{
	struct {
		__le64 pkt_addr; /* Packet buffer address */
		__le64 hdr_addr; /* Header buffer address */
	} read;
	struct {
		struct {
			union {
				__le32 data;
				struct {
					__le16 pkt_info; /* RSS, Pkt type */
					__le16 hdr_info; /* Splithdr, hdrlen */
				} hs_rss;
			} lo_dword;
			union {
				__le32 rss; /* RSS Hash */
				struct {
					__le16 ip_id; /* IP id */
					__le16 csum; /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error; /* ext status/error */
			__le16 length; /* Packet length */
			__le16 vlan; /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

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

struct mnic_bar4{
 	struct mnic_rx_register *rx;
	struct mnic_tx_register *tx;	
	//uint32_t tx_desc_idx;
	//uint32_t rx_desc_idx;
	uint32_t enabled;
};

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

struct packet_buffer{
	struct memory_pool *memp;
	dma_addr_t paddr;
	uint32_t index;
	uint32_t size;
	uint8_t data[] __attribute_((aligned(64));
};

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
	dma_addr_t rx_desc_paddr;
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

#define mnic_get_mac(dst, src) do {			\
		dst[0] = src[5];		\
		dst[1] = src[4];		\
		dst[2] = src[3];		\
		dst[3] = src[2];		\
		dst[4] = src[1];		\
		dst[5] = src[0];		\
	}while(0)

#endif

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



