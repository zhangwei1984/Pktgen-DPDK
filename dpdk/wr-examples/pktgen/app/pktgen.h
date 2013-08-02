/*-
 * Copyright (c) <2010>, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of Intel Corporation nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright (c) <2010-2012>, Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * 4) The screens displayed by the application must contain the copyright notice as defined
 * above and can not be removed without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Created 2010 by Keith Wiles @ windriver.com */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <net/if.h>
#include <fcntl.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <libgen.h>
#include <linux/if_tun.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <assert.h>

#include <rte_version.h>
#include <rte_config.h>

#include <rte_errno.h>
#include <rte_log.h>
#include <rte_tailq.h>
#include <rte_common.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_malloc.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_pci.h>
#include <rte_random.h>
#include <rte_timer.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_scrn.h>

#include <wr_copyright_info.h>
#include <wr_l2p.h>
#include <wr_port_config.h>
#include <wr_core_info.h>

#include <cmdline_rdline.h>
#include <cmdline_parse.h>
#include <cmdline_socket.h>
#include <cmdline_parse_string.h>
#include <cmdline_parse_num.h>
#include <cmdline_parse_ipaddr.h>
#include <cmdline_parse_etheraddr.h>
#include <cmdline_parse_portlist.h>
#include <cmdline.h>

#include <wr_pcap.h>
#include <wr_inet.h>
#include <wr_cksum.h>

#include <wr_cycles.h>
#include <wr_mempool.h>
#include <wr_mbuf.h>
#include <wr_coremap.h>
#include <wr_lscpu.h>
#include <wr_utils.h>

#ifndef _PKTGEN_H_
#define _PKTGEN_H_

int main(int argc, char **argv);

#define PKTGEN_VERSION			"2.1.5"
#define PKTGEN_APP_NAME			"Pktgen"
#define PKTGEN_CREATED_BY		"Keith Wiles"

#define MAX_MATRIX_ENTRIES      128
#define MAX_STRING              256
#define Million					(uint64_t)(1000ULL * 1000ULL)
#define Mega					(uint64_t)(1024ULL * 1024ULL)

#define iBitsTotal(_x) \
    (((_x.ipackets * (INTER_FRAME_GAP + PKT_PREAMBLE_SIZE)) + _x.ibytes) << 3)
#define oBitsTotal(_x) \
    (((_x.opackets * (INTER_FRAME_GAP + PKT_PREAMBLE_SIZE)) + _x.obytes) << 3)

#undef PKTGEN_DEBUG
#ifdef PKTGEN_DEBUG
/*lint -emacro( {717}, dbgPrintf ) */
#define dbgPrintf( ... )	do { printf(__VA_ARGS__); fflush(stdout); } while((0))
#else
#define dbgPrintf( ... ) 	do { } while((0))
#endif

#define _do(_exp)		do { _exp; } while((0))

#define foreach_port(_portlist, _action)				\
	do {												\
		uint32_t		pid;							\
		for(pid = 0; pid < pktgen.nb_ports; pid++) {	\
			port_info_t	  * info;						\
			if ( (_portlist & (1 << pid)) == 0 )		\
				continue;								\
			info = &pktgen.info[pid];					\
			_action;									\
		}												\
	} while((0))

typedef enum { PACKET_CONSUMED = 0, UNKNOWN_PACKET = 0xEEEE, DROP_PACKET = 0xFFFE, FREE_PACKET = 0xFFFF } pktType_e;

enum {
	MAX_SCRN_ROWS			= 44,
	MAX_SCRN_COLS			= 132,

	COLUMN_WIDTH_0			= 18,
	COLUMN_WIDTH_1			= 18,

	// Row locations for start of data
	PORT_STATE_ROWS			= 1,
	LINK_STATE_ROWS			= 4,
	PKT_SIZE_ROWS			= 9,
	PKT_TOTALS_ROWS			= 8,
	IP_ADDR_ROWS			= 10,
		
	PORT_STATE_ROW			= 2,
	LINK_STATE_ROW			= (PORT_STATE_ROW + PORT_STATE_ROWS),
	PKT_SIZE_ROW			= (LINK_STATE_ROW + LINK_STATE_ROWS),
	PKT_TOTALS_ROW			= (PKT_SIZE_ROW + PKT_SIZE_ROWS),
	IP_ADDR_ROW				= (PKT_TOTALS_ROW + PKT_TOTALS_ROWS),
	
	DEFAULT_NETMASK			= 0xFFFFFF00,
	DEFAULT_IP_ADDR			= (192 << 24) | (168 << 16),
	DEFAULT_TX_COUNT		= 0,			// Forever
	DEFAULT_TX_RATE			= 100,
	DEFAULT_PRIME_COUNT		= 3,
	DEFAULT_SRC_PORT		= 1234,
	DEFAULT_DST_PORT		= 5678,
	DEFAULT_PKT_NUMBER		= 0x012345678,
	DEFAULT_ACK_NUMBER		= 0x012345690,
	DEFAULT_WND_SIZE		= 8192,
	MIN_VLAN_ID				= 1,
	MAX_VLAN_ID				= 4095,
	DEFAULT_VLAN_ID			= MIN_VLAN_ID,
	MAX_ETHER_TYPE_SIZE		= 0x600,
	OVERHEAD_FUDGE_VALUE	= 25,

	DEFAULT_BUFF_SIZE		= 2048,
	DEFAULT_PORTS_PER_PAGE	= 4,
	VLAN_TAG_SIZE			= 4,
	MAX_PRIME_COUNT			= 4,

	NUM_SEQ_PKTS			= 16,		// Number of buffers to support in sequence
	NUM_EXTRA_TX_PKTS		= 8,		// Number of extra TX packets

	FIRST_SEQ_PKT			= 0,
	SINGLE_PKT				= (FIRST_SEQ_PKT + NUM_SEQ_PKTS),		// 16
	RANGE_PKT				= (SINGLE_PKT + 1),						// 17
	PING_PKT				= (RANGE_PKT + 1),						// 18
	EXTRA_TX_PKT			= (PING_PKT + 1),						// 19
	NUM_TOTAL_PKTS			= (EXTRA_TX_PKT + NUM_EXTRA_TX_PKTS),
	
	DEFAULT_PKT_BURST		= 128,		// Increasing this number consumes memory very fast
	DEFAULT_RX_DESC			= (DEFAULT_PKT_BURST * 4),
	DEFAULT_TX_DESC			= DEFAULT_RX_DESC,
	MAX_MBUFS_PER_PORT		= (DEFAULT_TX_DESC * 2),	// number of buffers to support per port
	MAX_SPECIAL_MBUFS		= 64,

	MBUF_CACHE_SIZE			= (MAX_MBUFS_PER_PORT/8),

	POLL_RATE				= 1000,
	ALL_PORTS				= 0xFFFFFFFF,
	INTER_FRAME_GAP			= 12,
	PKT_PREAMBLE_SIZE		= 8,
	FCS_SIZE				= 4,
	MIN_PKT_SIZE			= (ETHER_MIN_LEN - FCS_SIZE),
	MAX_PKT_SIZE			= (ETHER_MAX_LEN - FCS_SIZE),
	
	MAX_RX_QUEUES			= 16,	/**< RX Queues per port */
	MAX_TX_QUEUES			= 16,	/**< TX Queues per port */

	RX_PTHRESH 				= 8,	/**< Default values of RX prefetch threshold reg. */
	RX_HTHRESH 				= 8,	/**< Default values of RX host threshold reg. */
	RX_WTHRESH 				= 4,	/**< Default values of RX write-back threshold reg. */

	TX_PTHRESH 				= 36,	/**< Default values of TX prefetch threshold reg. */
	TX_HTHRESH 				= 0,	/**< Default values of TX host threshold reg. */
	TX_WTHRESH 				= 0,	/**< Default values of TX write-back threshold reg. */
	TX_WTHRESH_1GB			= 16,	/**< Default value for 1GB ports */

	PCAP_PAGE_SIZE			= 25,	/**< Size of the PCAP display page */

	NUM_Q					= 8,	/**< Number of cores per port. */

	SOCKET0					= 0		/**< Socket ID value for allocation */
};

enum { DISABLE_STATE = 0, ENABLE_STATE = 1 };

typedef struct rte_eth_stats	eth_stats_t;

typedef struct port_sizes_s {
	uint64_t			_64;				/**< Number of 64 byte packets */
	uint64_t			_65_127;			/**< Number of 65-127 byte packets */
	uint64_t			_128_255;			/**< Number of 128-255 byte packets */
	uint64_t			_256_511;			/**< Number of 256-511 byte packets */
	uint64_t			_512_1023;			/**< Number of 512-1023 byte packets */
	uint64_t			_1024_1518;			/**< Number of 1024-1518 byte packets */
	uint64_t			broadcast;			/**< Number of broadcast packets */
	uint64_t			multicast;			/**< Number of multicast packets */
	uint64_t			jumbo;				/**< Number of Jumbo frames */
	uint64_t			runt;				/**< Number of Runt frames */
} port_sizes_t;

typedef struct pkt_stats_s {
	uint64_t			arp_pkts;			/**< Number of ARP packets received */
	uint64_t			echo_pkts;			/**< Number of ICMP echo requests received */
	uint64_t			ip_pkts;			/**< Number of IPv4 packets received */
	uint64_t			ipv6_pkts;			/**< Number of IPv6 packets received */
	uint64_t			vlan_pkts;			/**< Number of VLAN packets received */
	uint64_t			dropped_pkts;		/**< Hyperscan dropped packets */
	uint64_t			unknown_pkts;		/**< Number of Unknown packets */
	uint64_t			tx_failed;			/**< Transmits that failed to send */
} pkt_stats_t;

typedef struct rte_mbuf	rte_mbuf_t;

struct mbuf_table {
	unsigned len;
	struct rte_mbuf *m_table[DEFAULT_PKT_BURST+1];
};

typedef struct pkt_seq_s {
    // Packet type and information
    struct ether_addr   eth_dst_addr;           /**< Destination Ethernet address */
    struct ether_addr   eth_src_addr;           /**< Source Ethernet address */
   
    uint32_t            ip_src_addr;            /**< Source IPv4 address also used for IPv6 */
    uint32_t            ip_dst_addr;            /**< Destination IPv4 address */
    uint32_t            ip_mask;                /**< IPv4 Netmask value */

    uint16_t            sport;                  /**< Source port value */
    uint16_t            dport;                  /**< Destination port value */
    uint16_t            ethType;                /**< IPv4 or IPv6 */
    uint16_t            ipProto;                /**< TCP or UDP or ICMP */
    uint16_t			vlanid;					/**< VLAN ID value if used */
    uint16_t			ether_hdr_size;			/**< Size of Ethernet header in packet for VLAN ID */

    uint16_t            pktSize;                /**< Size of packet in bytes not counting FCS */
    uint16_t            tlen;					/**< Total length of packet data */
    											/* 28 bytes + (2 * sizeof(struct ether_addr)) */
    pkt_hdr_t			hdr;					/**< Packet header data */
    uint8_t				pad[DEFAULT_BUFF_SIZE - (sizeof(pkt_hdr_t) + (sizeof(struct ether_addr)*2) + 28)];
} pkt_seq_t;

typedef union {
	struct ether_addr	addr;
	uint64_t			u64;
} ethaddr_t;

typedef struct range_info_s {
	uint32_t				src_ip_inc;			/**< Source IP increment */
	uint32_t				dst_ip_inc;			/**< Destination increment IP address */
	uint16_t				src_port_inc;		/**< Source port increment */
	uint16_t				dst_port_inc;		/**< Destination port increment */
	uint16_t				vlan_id_inc;		/**< VLAN id increment */
	uint16_t				pkt_size_inc;		/**< PKT size increment */

	uint32_t				src_ip;				/**< Source starting IP address */
	uint32_t				src_ip_min;			/**< Source IP minimum */
	uint32_t				src_ip_max;			/**< Source IP maximum */

	uint32_t				dst_ip;				/**< Destination starting IP address */
	uint32_t				dst_ip_min;			/**< Destination minimum IP address */
	uint32_t				dst_ip_max;			/**< Destination maximum IP address */

	uint16_t				src_port;			/**< Source port starting */
	uint16_t				src_port_min;		/**< Source port minimum */
	uint16_t				src_port_max;		/**< Source port maximum */

	uint16_t				dst_port;			/**< Destination port starting */
	uint16_t				dst_port_min;		/**< Destination port minimum */
	uint16_t				dst_port_max;		/**< Destination port maximum */

	uint16_t				vlan_id;			/**< VLAN id starting */
	uint16_t				vlan_id_min;		/**< VLAN id minimum */
	uint16_t				vlan_id_max;		/**< VLAN id maximum */

	uint16_t				pkt_size;			/**< PKT Size starting */
	uint16_t				pkt_size_min;		/**< PKT Size minimum */
	uint16_t				pkt_size_max;		/**< PKT Size maximum */

	ethaddr_t				dst_mac;			/**< Destination Starting MAC address */
	ethaddr_t				src_mac;			/**< Source Starting MAC address */
} range_info_t;

typedef struct port_info_s {
	uint16_t				pid;				/**< Port ID value */
	uint16_t				tx_burst;			/**< Number of TX burst packets */
	uint8_t					transmitting;		/**< Is port transmitting */
	uint8_t					tx_rate;			/**< Percentage rate for tx packets */
	rte_atomic32_t			port_flags;			/**< Special send flags for ARP and other */

	uint64_t				transmit_count;		/**< Packets to transmit loaded into current_tx_count */
	uint64_t				current_tx_count;	/**< Current number of packets to send */
	uint64_t				tx_cycles;			/**< Number cycles between TX bursts */
	uint64_t				tx_pps;				/**< Transmit packets per seconds */
	uint64_t				delta;				/**< Delta value for latency testing */
	uint64_t				tx_count;			/**< Total count of tx attempts */
	
	// Packet buffer space for traffic generator, shared for all packets per port
	uint16_t				seqIdx;				/**< Current Packet sequence index 0 to NUM_SEQ_PKTS */
	uint16_t				seqCnt;				/**< Current packet sequence max count */
	uint16_t				prime_cnt;			/**< Set the number of packets to send in a prime command */
	uint16_t				vlanid;				/**< Set the port VLAN ID value */
	pkt_seq_t			  * seq_pkt;			/**< Sequence of packets seq_pkt[NUM_SEQ_PKTS]=default packet */
	range_info_t			range;				/**< Range Information */

	uint16_t				nb_mbufs;			/**< Number of mbufs in the system */
	uint16_t				pad0;
	
	pkt_stats_t				stats;				/**< Statistics for a number of stats */
	port_sizes_t			sizes;				/**< Stats for the different packets sizes */

	eth_stats_t				init_stats;			/**< Initial packet statistics */
	eth_stats_t				port_stats;			/**< current port statistics */
	eth_stats_t				rate_stats;			/**< current packet rate statistics */

	struct rte_eth_link		link;				/**< Link Information like speed and duplex */

	struct q_info {
		rte_atomic32_t		 flags;				/**< Special send flags for ARP and other */
		struct mbuf_table	 tx_mbufs;			/**< mbuf holder for transmit packets */
		struct rte_mempool * rx_mp;				/**< Pool pointer for port RX mbufs */
		struct rte_mempool * tx_mp;				/**< Pool pointer for default TX mbufs */
		struct rte_mempool * range_mp;			/**< Pool pointer for port Range TX mbufs */
		struct rte_mempool * seq_mp;			/**< Pool pointer for port Sequence TX mbufs */
		struct rte_mempool * pcap_mp;			/**< Pool pointer for port PCAP TX mbufs */
		struct rte_mempool * special_mp;		/**< Pool pointer for special TX mbufs */
	} q[NUM_Q];

	int32_t					tapfd;				/**< Tap file descriptor */
	pcap_info_t			  * pcap;				/**< PCAP information header */
	uint64_t				pcap_cycles;		/**< number of cycles for pcap sending */
} port_info_t;

#define MAX_PORT_DESC_SIZE	132

/* Ethernet addresses of ports */
typedef struct pktgen_s {
	struct cmdline		  * cl;					/**< Command Line information pointer */
	char				  * cmd_filename;		/**< Command file path and name */
	void				  * L;					/**< Lua State pointer */
	rte_scrn_t			  * scrn;				/**< Screen structure pointer */
	char				  * hostname;			/**< GUI hostname */
	int32_t					socket_port;		/**< GUI port number */
	uint32_t				coremask;			/**< Coremask of lcores */
	char				  * prompt;				/**< Pktgen command line prompt */
	
	uint32_t				enabled_port_mask;	/**< mask of enabled ports */
	uint32_t				blinklist;			/**< Port list for blinking the led */
	uint32_t				flags;				/**< Flag values */
	uint16_t				ident;				/**< IPv4 ident value */
	uint16_t				last_row;			/**< last static row of the screen */
	uint16_t				nb_ports;			/**< Number of ports in the system */
	uint8_t					starting_port;		/**< Starting port to display */
	uint8_t					ending_port;		/**< Ending port to display */
	uint8_t					nb_ports_per_page;	/**< Number of ports to display per page */

	uint16_t				nb_rxd;				/**< Number of receive descriptors */
	uint16_t				nb_txd;				/**< Number of transmit descriptors */
	uint16_t				portNum;			/**< Current Port number */
	uint16_t				port_cnt;			/**< Number of ports used in total */
	uint64_t				hz;					/**< Number of events per seconds */

	int						(*callout)(void * callout_arg);
	void				  * callout_arg;

	struct rte_pci_addr		blacklist[RTE_MAX_ETHPORTS];
	struct rte_pci_addr		portlist[RTE_MAX_ETHPORTS];
	uint8_t				  * portdesc[RTE_MAX_ETHPORTS];
	uint32_t				portdesc_cnt;
	uint32_t				blacklist_cnt;

	// port to lcore mapping
	l2p_t				  * l2p;
	
	port_info_t				info[RTE_MAX_ETHPORTS];	/**< Port information */
	lc_info_t				core_info[RTE_MAX_LCORE];
	uint32_t				core_cnt;
	lscpu_t				  * lscpu;
	char				  * uname;
	eth_stats_t				cumm_rate_totals;	/**< port rates total values */

	pthread_t				thread;				/**< Thread structure for Lua server */

	uint64_t				counter;			/**< A debug counter */
	uint64_t				mem_used;			/**< Display memory used counters per ports */
	uint64_t				total_mem_used;		/**< Display memory used for all ports */
	int32_t					argc;				/**< Number of arguments */
	char				  * argv[64];			/**< Argument list */
} pktgen_t;

enum {		// Per port flag bits
	SEND_ARP_REQUEST		= 0x00000001,		/**< Send a ARP request */
	SEND_GRATUITOUS_ARP		= 0x00000002,		/**< Send a Gratuitous ARP */
	ICMP_ECHO_ENABLE_FLAG	= 0x00000004,		/**< Enable ICMP Echo support */
	SEND_PCAP_PKTS			= 0x00000008,		/**< Send a pcap file of packets */
	SEND_RANGE_PKTS			= 0x00000010,		/**< Send a range of packets */
	SEND_SEQ_PKTS			= 0x00000020,		/**< Send a sequence of packets */
	PROCESS_INPUT_PKTS		= 0x00000040,		/**< Process input packets */
	SEND_PING4_REQUEST		= 0x00000080,		/**< Send a IPv4 Ping request */
	SEND_PING6_REQUEST		= 0x00000100,		/**< Send a IPv6 Ping request */
	SEND_SPECIAL_REQUEST	= (SEND_ARP_REQUEST | SEND_GRATUITOUS_ARP | SEND_PING4_REQUEST | SEND_PING6_REQUEST),
	PROCESS_TAP_PKTS		= 0x00000200,		/**< Handle TAP interface packets */
	SEND_VLAN_ID			= 0x00000400		/**< Send packets with VLAN ID */
};

enum {	// Queue flags
	DO_TX_CLEANUP			= 0x00000001,		/**< Do a TX cleanup */
	CLEAR_FAST_ALLOC_FLAG	= 0x00000002,		/**< Clear the TX fast alloc flag */
	DO_TX_FLUSH				= 0x00008004		/**< Do a TX Flush by sending all of the pkts in the queue */
};

enum {		// Pktgen flags bits
	PRINT_LABELS_FLAG		= 0x00000001,		/**< Print constant labels on stats display */
	MAC_FROM_ARP_FLAG		= 0x00000002,		/**< Configure the SRC MAC from a ARP request */
	PROMISCUOUS_ON_FLAG		= 0x00000004,		/**< Enable promiscuous mode */
	NUMA_SUPPORT_FLAG		= 0x00000008,		/**< Enable NUMA support */
	CONFIG_PAGE_FLAG		= 0x00000010,		/**< Display the configure page */
	SEQUENCE_PAGE_FLAG		= 0x00000020,		/**< Display the Packet sequence page */
	FAKE_PORTS_FLAG			= 0x00000040,		/**< Fake ports enabled */
	BLINK_PORTS_FLAG		= 0x00000080,		/**< Blink the port leds */
	RANGE_PAGE_FLAG			= 0x00000100,		/**< Display the range page */
	PCAP_PAGE_FLAG			= 0x00000200,		/**< Display the PCAP page */
	CPU_PAGE_FLAG			= 0x00000400,		/**< Display the PCAP page */
	IS_SERVER_FLAG			= 0x00000800,		/**< Pktgen is a Server */
	ENABLE_GUI_FLAG			= 0x00001000,		/**< GUI support is enabled */
	LUA_SHELL_FLAG			= 0x00002000		/**< Enable Lua Shell */
};

struct cmdline_etheraddr {
	uint8_t	mac[6];
};
typedef struct cmdline_etheraddr cmdline_etheraddr_t;

extern pktgen_t		pktgen;

static __inline__ void
pktgen_set_port_flags(port_info_t * info, uint32_t flags) {
	uint32_t	val;

    do {
    	val = rte_atomic32_read(&info->port_flags);
    } while( rte_atomic32_cmpset(&info->port_flags.cnt, val, (val | flags)) == 0 );
}

static __inline__ void
pktgen_clr_port_flags(port_info_t * info, uint32_t flags) {
	uint32_t	val;

    do {
    	val = rte_atomic32_read(&info->port_flags);
    } while( rte_atomic32_cmpset(&info->port_flags.cnt, val, (val & ~flags)) == 0 );
}

static __inline__ void
pktgen_set_q_flags(port_info_t * info, uint8_t q, uint32_t flags) {
	uint32_t	val;

    do {
    	val = rte_atomic32_read(&info->q[q].flags);
    } while( rte_atomic32_cmpset(&info->q[q].flags.cnt, val, (val | flags)) == 0 );
}

static __inline__ void
pktgen_clr_q_flags(port_info_t * info, uint8_t q, uint32_t flags) {
	uint32_t	val;

    do {
    	val = rte_atomic32_read(&info->q[q].flags);
    } while( rte_atomic32_cmpset(&info->q[q].flags.cnt, val, (val & ~flags)) == 0 );
}

/**
 * Function returning string of version number: "- Version:x.y.x (DPDK-x.y.z)"
 * @return
 *     string
 */
static inline const char *
pktgen_version(void) {
	return "Ver:"PKTGEN_VERSION"(DPDK-"
			RTE_STR(RTE_VER_MAJOR)"."
			RTE_STR(RTE_VER_MINOR)"."
			RTE_STR(RTE_VER_PATCH_LEVEL)")";
}

static __inline__ uint32_t
parseState(const char * state) {
	return ( !strcasecmp(state, "on") || !strcasecmp(state, "enable") || !strcasecmp(state, "start") ) ?
			ENABLE_STATE : DISABLE_STATE;
}

static __inline__ char *
strdupf(char * str, char * new) {
	if ( str ) free(str);
	return (new == NULL) ? NULL : strdup(new);
}

#define printf_info(...)	scrn_fprintf(0,0,stdout, __VA_ARGS__)

extern void pktgen_stop_running(void);
extern void pktgen_send_seq_pkt(port_info_t * info, uint32_t seqnum);

#endif /* _PKTGEN_H_ */

#include "pktgen-cmds.h"

#define lua_c
#include <lua.h>
#include "lualib.h"
#include "lauxlib.h"
#include "lua-socket.h"
#include "lpktgenlib.h"

