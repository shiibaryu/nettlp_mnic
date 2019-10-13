union nettlp_mnic_tx_desc{
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

union nettlp_mnic_rx_desc {
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
	uint64_t rdba; //receive base address registers
	uint64_t rdlen; //receive descriptor length registers
	uint64_t rdh; //receive descriptor head
	uint64_t rdt; //receive descriptor tail
};

struct mnic_tx_register{
	uint64_t tdba; //transmit descriptor base address
	uint64_t tdlen; //transmit descirptor len
	uint64_t tdh; //transmit descriptor head
	uint64_t tdt; //transmit descriptor tail
};

struct mnic_bar4{
	struct mnic_rx_register *mrr;
	struct mnic_tx_register *mtr;	
	uint32_t tx_desc_idx;
	uint32_t rx_desc_idx;
	uint32_t enabled;
}__attribute__((packed));

struct nettlp_mnic{
	volatile union nettlp_mnic_tx_desc *nmtd;//tx descriptor(max queue 4096,min queue 512)
	volatile union nettlp_mnic_rx_desc *nmrd;//rx descriptor(max queue 4096,min queue 512)
	struct mnic_bar4 mbar4;
};

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



