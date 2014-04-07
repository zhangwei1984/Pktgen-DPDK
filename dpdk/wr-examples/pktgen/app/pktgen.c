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
 * Copyright (c) <2010-2014>, Wind River Systems, Inc.
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

#include "pktgen.h"

#ifndef MEMPOOL_F_DMA
#define MEMPOOL_F_DMA       0
#endif

enum {
    MBUF_SIZE   = (DEFAULT_BUFF_SIZE - sizeof(struct rte_mbuf))
};

// Allocated the pktgen structure for global use
    pktgen_t        pktgen;

/*
 * Receive Side Scaling (RSS) configuration.
 */
uint16_t rss_hf = ETH_RSS_IPV4 | ETH_RSS_IPV6; /* RSS IP by default. */

const struct rte_eth_conf port_conf = {
    .rxmode = {
        .split_hdr_size = 0,
        .header_split   = 0, /**< Header Split disabled */
        .hw_ip_checksum = 0, /**< IP checksum offload disabled */
        .hw_vlan_filter = 0, /**< VLAN filtering disabled */
        .jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
    },
    .txmode = {
    },
};

const struct rte_eth_rxconf rx_conf = {
    .rx_thresh = {
        .pthresh = RX_PTHRESH,
        .hthresh = RX_HTHRESH,
        .wthresh = RX_WTHRESH,
    },
    .rx_free_thresh = 32,
};

#define IXGBE_SIMPLE_FLAGS ((uint32_t)ETH_TXQ_FLAGS_NOMULTSEGS | \
			    ETH_TXQ_FLAGS_NOOFFLOADS)

const struct rte_eth_txconf tx_conf = {
    .tx_thresh = {
        .pthresh = TX_PTHRESH,
        .hthresh = TX_HTHRESH,
        .wthresh = TX_WTHRESH,
    },
    .tx_rs_thresh = 0,
    .tx_free_thresh = 0, /* Use PMD default values */
    .txq_flags = IXGBE_SIMPLE_FLAGS,
};
#if 0
static struct rte_eth_fc_conf fc_conf = {
    .mode       = RTE_FC_NONE, //RTE_FC_TX_PAUSE
    .high_water = 80 * 510 / 100,
    .low_water  = 60 * 510 / 100,
    .pause_time = 1337,
    .send_xon   = 0,
};
#endif
// Forward declaration of functions.
static void pktgen_process_arp( struct rte_mbuf * m, uint32_t pid, uint32_t vlan );
static void pktgen_process_ping4( struct rte_mbuf * m, uint32_t pid, uint32_t vlan);
static void pktgen_process_ping6( struct rte_mbuf * m, uint32_t pid, uint32_t vlan );
static void pktgen_process_vlan( struct rte_mbuf * m, uint32_t pid );

/**************************************************************************//**
*
* do_command - Internal function to execute a shell command and grab the output.
*
* DESCRIPTION
* Internal function to execute a shell command and grab the output from the command.
*
* RETURNS: Nubmer of lines read.
*
* SEE ALSO:
*/

static __inline__ int do_command(const char * cmd, int (*display)(char *, int)) {
	FILE	  * f;
	int			i;
	char * line = NULL;
	size_t	line_size = 0;

	f = popen(cmd, "r");
	if ( f == NULL ) {
		printf("Unable to run '%s' command\n", cmd);
		return -1;
	}

	i = 0;
	while(getline(&line, &line_size, f) > 0)
		i = display(line, i);

	if ( f ) fclose(f);
	if ( line ) free(line);

	return i;
}

/**************************************************************************//**
*
* pktgen_wire_size - Calculate the wire size of the data to be sent.
*
* DESCRIPTION
* Calculate the number of bytes/bits in a burst of traffic.
*
* RETURNS: Number of bits in burst of packets.
*
* SEE ALSO:
*/

static __inline__ uint64_t
pktgen_wire_size( port_info_t * info ) {
    uint64_t    i, size = 0;

	if ( rte_atomic32_read(&info->port_flags) & SEND_PCAP_PKTS )
		size = info->pcap->pkt_size + PKT_PREAMBLE_SIZE + INTER_FRAME_GAP + FCS_SIZE;
	else {
		if ( unlikely(info->seqCnt > 0) ) {
			for(i = 0; i < info->seqCnt; i++)
				size += info->seq_pkt[i].pktSize + PKT_PREAMBLE_SIZE + INTER_FRAME_GAP + FCS_SIZE;
			size = size / info->seqCnt;		// Calculate the average sized packet
		} else
			size = info->seq_pkt[SINGLE_PKT].pktSize + PKT_PREAMBLE_SIZE + INTER_FRAME_GAP + FCS_SIZE;
	}
    return size;
}

/**************************************************************************//**
*
* pktgen_packet_rate - Calculate the transmit rate.
*
* DESCRIPTION
* Calculate the number of cycles to wait between sending bursts of traffic.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_packet_rate(port_info_t * info)
{
                          /*   0   1   2   3   4   5   6   7   8   9  10 */
	static int64_t ff[11] = { 31, 30, 25, 30, 17, 17, 17, 20, 50, 60, 90 };
    uint64_t    wire_size = (pktgen_wire_size(info) * 8);
    uint64_t	link = (uint64_t)info->link.link_speed * Million;
    uint64_t    pps = ((link/wire_size) * info->tx_rate)/100;
    uint64_t	cpp = (pps > 0) ? (pktgen.hz/pps) : (pktgen.hz / 4);

    info->tx_pps		= pps;
    info->tx_cycles 	= ((cpp * info->tx_burst) / wr_get_port_txcnt(pktgen.l2p, info->pid));
	info->tx_cycles		-= ff[info->tx_rate/10];
}

/**************************************************************************//**
*
* pktgen_fill_pattern - Create the fill pattern in a packet buffer.
*
* DESCRIPTION
* Create a fill pattern based on the arguments for the packet data.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_fill_pattern( uint8_t * p, uint32_t len, uint32_t type ) {
    uint32_t    i;

    switch(type) {
    case 1:                 // Byte wide ASCII pattern
        for(i = 0; i < len; i++)
            p[i] = "abcdefghijklmnopqrstuvwxyz012345"[i & 0x1f];
        break;
    default: memset(p, 0, len); break;
    }
}

/**************************************************************************//**
*
* pktgen_find_matching_ipsrc - Find the matching IP source address
*
* DESCRIPTION
* locate and return the pkt_seq_t pointer to the match IP address.
*
* RETURNS: pkt_seq_t  * or NULL
*
* SEE ALSO:
*/

static __inline__ pkt_seq_t *
pktgen_find_matching_ipsrc( port_info_t * info, uint32_t addr )
{
	pkt_seq_t * pkt = NULL;
	int		i;

	addr = ntohl(addr);

	/* Search the sequence packets for a match */
	for(i = 0; i < info->seqCnt; i++) {
		if ( addr == info->seq_pkt[i].ip_src_addr ) {
			pkt = &info->seq_pkt[i];
			break;
		}
	}

	/* Now try to match the single packet address */
	if ( pkt == NULL ) {
		if ( addr == info->seq_pkt[SINGLE_PKT].ip_src_addr )
			pkt = &info->seq_pkt[SINGLE_PKT];
	}

	return pkt;
}

/**************************************************************************//**
*
* pktgen_find_matching_ipdst - Find the matching IP destination address
*
* DESCRIPTION
* locate and return the pkt_seq_t pointer to the match IP address.
*
* RETURNS: pkt_seq_t  * or NULL
*
* SEE ALSO:
*/

static __inline__ pkt_seq_t *
pktgen_find_matching_ipdst( port_info_t * info, uint32_t addr ) 
{
	pkt_seq_t * pkt = NULL;
	int		i;

	addr = ntohl(addr);

	/* Search the sequence packets for a match */
	for(i = 0; i < info->seqCnt; i++) {
		if ( addr == info->seq_pkt[i].ip_dst_addr ) {
			pkt = &info->seq_pkt[i];
			break;
		}
	}

	/* Now try to match the single packet address */
	if ( pkt == NULL ) {
		if ( addr == info->seq_pkt[SINGLE_PKT].ip_dst_addr )
			pkt = &info->seq_pkt[SINGLE_PKT];
	}

	/* Now try to match the range packet address */
	if ( pkt == NULL ) {
		if ( addr == info->seq_pkt[RANGE_PKT].ip_dst_addr )
			pkt = &info->seq_pkt[RANGE_PKT];
	}

	return pkt;
}

/**************************************************************************//**
*
* pktgen_send_burst - Send a burst of packets.
*
* DESCRIPTION
* Transmit a burst of packets to a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_send_burst(port_info_t * info, uint8_t qid)
{
	struct mbuf_table	* mtab = &info->q[qid].tx_mbufs;
	struct rte_mbuf **pkts = mtab->m_table;
    uint32_t ret, cnt, i, flags;

    if ( (cnt = mtab->len) == 0 )
    	return;

	flags = rte_atomic32_read(&info->port_flags);
    mtab->len = 0;
	do {
    	ret = rte_eth_tx_burst(info->pid, qid, pkts, cnt);
		if ( unlikely(flags & PROCESS_TX_TAP_PKTS) ) {
			for(i = 0; i < ret; i++) {
				if ( write(info->tx_tapfd, rte_pktmbuf_mtod(pkts[i], char *), pkts[i]->pkt.pkt_len) < 0 )
					printf_info("Write failed for tx_tap%d", info->pid);
			}
		}
		pkts += ret;
		cnt -= ret;
	} while( cnt > 0 );
}

/**************************************************************************//**
*
* pktgen_tx_flush - Flush Tx buffers from ring.
*
* DESCRIPTION
* Flush TX buffers from ring.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_tx_flush(port_info_t * info, uint8_t qid)
{
	// Flush any queued pkts to the driver.
	if ( unlikely(info->q[qid].tx_mbufs.len) )
		pktgen_send_burst(info, qid);

	pktgen_clr_q_flags(info, qid, DO_TX_FLUSH);
}

/**************************************************************************//**
*
* pktgen_tx_cleanup - Handle the transmit cleanup and flush tx buffers.
*
* DESCRIPTION
* Routine to force the tx done routine to cleanup transmit buffers.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_tx_cleanup(port_info_t * info, uint8_t qid)
{
	// Flush any done transmit buffers and descriptors.
	pktgen_tx_flush(info, qid);

	// Stop and start the device to flush TX and RX buffers from the device rings.
	rte_eth_dev_stop(info->pid);

	rte_eth_dev_start(info->pid);

	pktgen_clr_q_flags(info, qid, DO_TX_CLEANUP);
}

/**************************************************************************//**
*
* pktgen_cleanup - Clean up the hyperscan data and other items
*
* DESCRIPTION
* Clean up the hyperscan data.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_cleanup(uint8_t lid)
{
	port_info_t	* info;
	uint8_t		idx, pid;

	for( idx = 0; idx < wr_get_lcore_txcnt(pktgen.l2p, lid); idx++ ) {
		pid = wr_get_tx_pid(pktgen.l2p, lid, idx);
		if ( (info = (port_info_t *)wr_get_port_private(pktgen.l2p, pid)) != NULL )
			pktgen_tx_cleanup(info, wr_get_txque(pktgen.l2p, lid, pid));
	}
}

/**************************************************************************//**
*
* pktgen_has_work - Determine if lcore has work to do, if not wait for stop.
*
* DESCRIPTION
* If lcore has work to do then return zero else spin till stopped and return 1.
*
* RETURNS: 0 or 1
*
* SEE ALSO:
*/

static __inline__ int pktgen_has_work(void) {

	if ( ! wr_get_map(pktgen.l2p, RTE_MAX_ETHPORTS, rte_lcore_id()) ) {
        printf_info("*** Nothing to do on lcore %d Exiting\n", rte_lcore_id());
        return 1;
    }
    return 0;
}

/**************************************************************************//**
*
* pktgen_ether_hdr_ctor - Ethernet header constructor routine.
*
* DESCRIPTION
* Construct the ethernet header for a given packet buffer.
*
* RETURNS: Pointer to memory after the ethernet header.
*
* SEE ALSO:
*/

static __inline__ char *
pktgen_ether_hdr_ctor(port_info_t * info, pkt_seq_t * pkt, struct ether_hdr * eth)
{
    struct vlan_hdr *vlan_hdr;
	uint32_t	flags;

    /* src and dest addr */
    ether_addr_copy(&pkt->eth_src_addr, &eth->s_addr);
    ether_addr_copy(&pkt->eth_dst_addr, &eth->d_addr);

    flags = rte_atomic32_read(&info->port_flags);
    if ( flags & SEND_VLAN_ID ) {
		/* vlan ethernet header */
		eth->ether_type = htons(ETHER_TYPE_VLAN);

		/* only set the TCI field for now; don't bother with PCP/DEI */
		struct vlan_hdr *vlan_hdr = (struct vlan_hdr *)(eth+1);
		vlan_hdr->vlan_tci = htons(pkt->vlanid);
		vlan_hdr->eth_proto = htons(pkt->ethType);

		/* adjust header size for VLAN tag */
		pkt->ether_hdr_size = sizeof(struct ether_hdr) + sizeof(struct vlan_hdr);

		return (char *)(vlan_hdr+1);
    }
    else {
        /* normal ethernet header */
        eth->ether_type = htons(pkt->ethType);
        pkt->ether_hdr_size = sizeof(struct ether_hdr);
    }

    return (char *)(eth+1);
}

/**************************************************************************//**
*
* pktgen_ipv4_ctor - Construct the IPv4 header for a packet
*
* DESCRIPTION
* Constructor for the IPv4 header for a given packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_ipv4_ctor(pkt_seq_t * pkt, ipHdr_t * ip)
{
	uint16_t	tlen;

    // IPv4 Header constructor
    tlen                = pkt->pktSize - pkt->ether_hdr_size;

    // Zero out the header space
    memset((char *)ip, 0, sizeof(ipHdr_t));

    ip->vl              = (IPv4_VERSION << 4) | (sizeof(ipHdr_t) /4);

    ip->tlen            = htons(tlen);
    ip->ttl             = 4;
    ip->tos             = 0;

    pktgen.ident        += 27;          // bump by a prime number
    ip->ident           = htons(pktgen.ident);
    ip->ffrag           = 0;
    ip->proto           = pkt->ipProto;
    ip->src             = htonl(pkt->ip_src_addr);
    ip->dst             = htonl(pkt->ip_dst_addr);
    ip->cksum           = cksum(ip, sizeof(ipHdr_t), 0);
}

/**************************************************************************//**
*
* pktgen_ipv6_ctor - IPv6 packet header constructor routine.
*
* DESCRIPTION
* Construct the IPv6 header constructor routine.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_ipv6_ctor(pkt_seq_t * pkt, ipv6Hdr_t * ip)
{
    uint32_t            addr;
    uint16_t			tlen;

    // IPv6 Header constructor
    memset(ip, 0, sizeof(ipv6Hdr_t));

    ip->ver_tc_fl       = htonl(IPv6_VERSION << 28);
    tlen           		= pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipv6Hdr_t));

    ip->payload_length  = htons(tlen);
    ip->hop_limit       = 4;
    ip->next_header     = pkt->ipProto;

    addr                = htonl(pkt->ip_dst_addr);
    rte_memcpy(&ip->daddr[8], &addr, sizeof(uint32_t));
    addr                = htonl(pkt->ip_src_addr);
    rte_memcpy(&ip->saddr[8], &addr, sizeof(uint32_t));
}

/**************************************************************************//**
*
* pktgen_tcp_hdr_ctor - TCP header constructor routine.
*
* DESCRIPTION
* Construct a TCP header in the packet buffer provided.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_tcp_hdr_ctor(pkt_seq_t * pkt, tcpip_t * tip, __attribute__ ((unused)) int type)
{
	uint16_t		tlen;

    // Zero out the header space
    memset((char *)tip, 0, sizeof(tcpip_t));

    // Create the TCP header
    tip->ip.src         = htonl(pkt->ip_src_addr);
    tip->ip.dst         = htonl(pkt->ip_dst_addr);
    tlen           		= pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipHdr_t));

    tip->ip.len         = htons(tlen);
    tip->ip.proto       = pkt->ipProto;

    tip->tcp.sport      = htons(pkt->sport);
    tip->tcp.dport      = htons(pkt->dport);
    tip->tcp.seq        = htonl(DEFAULT_PKT_NUMBER);
    tip->tcp.ack        = htonl(DEFAULT_ACK_NUMBER);
    tip->tcp.offset     = ((sizeof(tcpHdr_t)/sizeof(uint32_t)) << 4);   /* Offset in words */
    tip->tcp.flags      = ACK_FLAG;     /* ACK */
    tip->tcp.window     = htons(DEFAULT_WND_SIZE);
    tip->tcp.urgent     = 0;

    tlen           		= pkt->pktSize - pkt->ether_hdr_size;

    tip->tcp.cksum      = cksum(tip, tlen, 0);
}

/**************************************************************************//**
*
* pktgen_udp_hdr_ctor - UDP header constructor routine.
*
* DESCRIPTION
* Construct the UDP header in a packer buffer.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_udp_hdr_ctor(pkt_seq_t * pkt, udpip_t * uip, __attribute__ ((unused)) int type)
{
	uint16_t		tlen;

    // Zero out the header space
    memset((char *)uip, 0, sizeof(udpip_t));

    // Create the UDP header
    uip->ip.src         = htonl(pkt->ip_src_addr);
    uip->ip.dst         = htonl(pkt->ip_dst_addr);
    tlen                = pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipHdr_t));

    uip->ip.len         = htons(tlen);
    uip->ip.proto       = pkt->ipProto;

	uip->udp.len		= htons(tlen);
    uip->udp.sport      = htons(pkt->sport);
    uip->udp.dport      = htons(pkt->dport);

	// Includes the pseudo header information
    tlen                = pkt->pktSize - pkt->ether_hdr_size;

    uip->udp.cksum      = cksum(uip, tlen, 0);
    if ( uip->udp.cksum == 0 )
        uip->udp.cksum = 0xFFFF;
}

/**************************************************************************//**
*
* pktgen_packet_ctor - Construct a complete packet with all headers and data.
*
* DESCRIPTION
* Construct a packet type based on the arguments passed with all headers.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_packet_ctor(port_info_t * info, int32_t seq_idx, int32_t type) {
	pkt_seq_t		  * pkt = &info->seq_pkt[seq_idx];
	struct ether_hdr  * eth = (struct ether_hdr *)&pkt->hdr.eth;
	uint16_t			tlen;

    // Fill in the pattern for data space.
    pktgen_fill_pattern((uint8_t *)&pkt->hdr, (sizeof(pkt_hdr_t) + sizeof(pkt->pad)), 1);

    if ( likely(pkt->ethType == ETHER_TYPE_IPv4) ) {

		if ( likely(pkt->ipProto == PG_IPPROTO_TCP) ) {
			tcpip_t	  * tip;

			// Construct the Ethernet header
			tip = (tcpip_t *)pktgen_ether_hdr_ctor(info, pkt, eth);

			// Construct the TCP header
			pktgen_tcp_hdr_ctor(pkt, tip, ETHER_TYPE_IPv4);

			// IPv4 Header constructor
			pktgen_ipv4_ctor(pkt, (ipHdr_t *)tip);

			pkt->tlen = pkt->ether_hdr_size + sizeof(ipHdr_t) + sizeof(tcpHdr_t);

		} else if ( (pkt->ipProto == PG_IPPROTO_UDP) ) {
			udpip_t	  * udp;

			// Construct the Ethernet header
			udp = (udpip_t *)pktgen_ether_hdr_ctor(info, pkt, eth);

			// Construct the UDP header
			pktgen_udp_hdr_ctor(pkt, udp, ETHER_TYPE_IPv4);

			// IPv4 Header constructor
			pktgen_ipv4_ctor(pkt, (ipHdr_t *)udp);

			pkt->tlen = pkt->ether_hdr_size + sizeof(ipHdr_t) + sizeof(udpHdr_t);

		} else if ( (pkt->ipProto == PG_IPPROTO_ICMP) ) {
			udpip_t           * uip;
			icmpv4Hdr_t       * icmp;

			// Construct the Ethernet header
			uip = (udpip_t *)pktgen_ether_hdr_ctor(info, pkt, eth);

			// Create the ICMP header
			uip->ip.src         = htonl(pkt->ip_src_addr);
			uip->ip.dst         = htonl(pkt->ip_dst_addr);
			tlen           		= pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipHdr_t));
			uip->ip.len         = htons(tlen);
			uip->ip.proto       = pkt->ipProto;

			icmp = (icmpv4Hdr_t *)&uip->udp;
			icmp->code                      = 0;
			if ( (type == -1) || (type == ICMP4_TIMESTAMP) ) {
				icmp->type                      = ICMP4_TIMESTAMP;
				icmp->data.timestamp.ident      = 0x1234;
				icmp->data.timestamp.seq        = 0x5678;
				icmp->data.timestamp.originate  = 0x80004321;
				icmp->data.timestamp.receive    = 0;
				icmp->data.timestamp.transmit   = 0;
			} else if ( type == ICMP4_ECHO ) {
				icmp->type                      = ICMP4_ECHO;
				icmp->data.echo.ident      		= 0x1234;
				icmp->data.echo.seq        		= 0x5678;
				icmp->data.echo.data			= 0;
			}
			icmp->cksum     = 0;
			tlen       		= pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipHdr_t)); //ICMP4_TIMESTAMP_SIZE
			icmp->cksum     = cksum(icmp, tlen, 0);
			if ( icmp->cksum == 0 )
				icmp->cksum = 0xFFFF;

			// IPv4 Header constructor
			pktgen_ipv4_ctor(pkt, (ipHdr_t *)uip);

			pkt->tlen = pkt->ether_hdr_size + sizeof(ipHdr_t) + ICMP4_TIMESTAMP_SIZE;
		}
    } else if ( pkt->ethType == ETHER_TYPE_IPv6 ) {
		if ( (pkt->ipProto == PG_IPPROTO_TCP) ) {
			uint32_t            addr;
			tcpipv6_t         * tip;

			// Construct the Ethernet header
			tip = (tcpipv6_t *)pktgen_ether_hdr_ctor(info, pkt, eth);

			// Create the pseudo header and TCP information
			addr                = htonl(pkt->ip_dst_addr);
			rte_memcpy(&tip->ip.daddr[8], &addr, sizeof(uint32_t));
			addr                = htonl(pkt->ip_src_addr);
			rte_memcpy(&tip->ip.saddr[8], &addr, sizeof(uint32_t));

			tlen           		= sizeof(tcpHdr_t) + (pkt->pktSize - pkt->ether_hdr_size - sizeof(ipv6Hdr_t) - sizeof(tcpHdr_t));
			tip->ip.tcp_length  = htonl(tlen);
			tip->ip.next_header = pkt->ipProto;

			tip->tcp.sport      = htons(pkt->sport);
			tip->tcp.dport      = htons(pkt->dport);
			tip->tcp.seq        = htonl(DEFAULT_PKT_NUMBER);
			tip->tcp.ack        = htonl(DEFAULT_ACK_NUMBER);
			tip->tcp.offset     = ((sizeof(tcpHdr_t)/sizeof(uint32_t)) << 4);   /* Offset in words */
			tip->tcp.window     = htons(DEFAULT_WND_SIZE);
			tip->tcp.urgent     = 0;
			tip->tcp.flags      = ACK_FLAG;     /* ACK */

			tlen           		= sizeof(tcpipv6_t) + (pkt->pktSize - pkt->ether_hdr_size - sizeof(ipv6Hdr_t) - sizeof(tcpHdr_t));
			tip->tcp.cksum      = cksum(tip, tlen, 0);

			// IPv6 Header constructor
			pktgen_ipv6_ctor(pkt, (ipv6Hdr_t *)&tip->ip);

			pkt->tlen = sizeof(tcpHdr_t) + pkt->ether_hdr_size + sizeof(ipv6Hdr_t);
			if ( unlikely(pkt->pktSize < pkt->tlen) )
				pkt->pktSize = pkt->tlen;

		} else if ( (pkt->ipProto == PG_IPPROTO_UDP) ) {
			uint32_t            addr;
			udpipv6_t         * uip;

			// Construct the Ethernet header
			uip = (udpipv6_t *)pktgen_ether_hdr_ctor(info, pkt, eth);

			// Create the pseudo header and TCP information
			addr                = htonl(pkt->ip_dst_addr);
			rte_memcpy(&uip->ip.daddr[8], &addr, sizeof(uint32_t));
			addr                = htonl(pkt->ip_src_addr);
			rte_memcpy(&uip->ip.saddr[8], &addr, sizeof(uint32_t));

			tlen           		= sizeof(udpHdr_t) + (pkt->pktSize - pkt->ether_hdr_size - sizeof(ipv6Hdr_t) - sizeof(udpHdr_t));
			uip->ip.tcp_length  = htonl(tlen);
			uip->ip.next_header = pkt->ipProto;

			uip->udp.sport      = htons(pkt->sport);
			uip->udp.dport      = htons(pkt->dport);

			tlen           		= sizeof(udpipv6_t) + (pkt->pktSize - pkt->ether_hdr_size - sizeof(ipv6Hdr_t) - sizeof(udpHdr_t));
			uip->udp.cksum      = cksum(uip, tlen, 0);
			if ( uip->udp.cksum == 0 )
				uip->udp.cksum = 0xFFFF;

			// IPv6 Header constructor
			pktgen_ipv6_ctor(pkt, (ipv6Hdr_t *)&uip->ip);

			pkt->tlen = sizeof(udpHdr_t) + pkt->ether_hdr_size + sizeof(ipv6Hdr_t);
			if ( unlikely(pkt->pktSize < pkt->tlen) )
				pkt->pktSize = pkt->tlen;
		}
    }
}

/**************************************************************************//**
*
* pktgen_send_mbuf - Send a single packet to the given port.
*
* DESCRIPTION
* Send a single packet to a given port, but enqueue the packet until we have
* a given burst count of packets to send.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_send_mbuf(struct rte_mbuf *m, uint8_t pid, uint8_t qid)
{
    port_info_t * info = &pktgen.info[pid];
	struct mbuf_table	* mtab = &info->q[qid].tx_mbufs;

    // Add packet to the TX list.
    mtab->m_table[mtab->len++] = m;

    /* Fill our tx burst requirement */
    if (unlikely(mtab->len == info->tx_burst))
        pktgen_send_burst(info, qid);
}

void
pktgen_send_seq_pkt(port_info_t * info, uint32_t seq_idx)
{
    (void)info;
    (void)seq_idx;
}

/**************************************************************************//**
*
* pktgen_packet_type - Examine a packet and return the type of packet
*
* DESCRIPTION
* Examine a packet and return the type of packet.
* the packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ pktType_e
pktgen_packet_type( struct rte_mbuf * m )
{
    pktType_e   ret;
    struct ether_hdr *eth;

    eth = rte_pktmbuf_mtod(m, struct ether_hdr *);

    ret = ntohs(eth->ether_type);

    return ret;
}

/**************************************************************************//**
*
* pktgen_packet_classify - Examine a packet and classify it for statistics
*
* DESCRIPTION
* Examine a packet and determine its type along with counting statistics around
* the packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_packet_classify( struct rte_mbuf * m, int pid )
{
    port_info_t * info = &pktgen.info[pid];
    int     plen = (m->pkt.pkt_len + FCS_SIZE);
	uint32_t	flags;
    pktType_e   pType;

    pType = pktgen_packet_type(m);

	flags = rte_atomic32_read(&info->port_flags);
	if ( unlikely(flags & (PROCESS_INPUT_PKTS | PROCESS_RX_TAP_PKTS)) ) {
		if ( unlikely(flags & PROCESS_RX_TAP_PKTS) ) {
			if ( write(info->rx_tapfd, rte_pktmbuf_mtod(m, char *), m->pkt.pkt_len) < 0 )
				printf_info("Write failed for rx_tap%d", pid);
		}

		switch((int)pType) {
		case ETHER_TYPE_ARP:	info->stats.arp_pkts++;		pktgen_process_arp(m, pid, 0);     break;
		case ETHER_TYPE_IPv4:   info->stats.ip_pkts++;		pktgen_process_ping4(m, pid, 0);   break;
		case ETHER_TYPE_IPv6:   info->stats.ipv6_pkts++;	pktgen_process_ping6(m, pid, 0);   break;
		case ETHER_TYPE_VLAN:   info->stats.vlan_pkts++;	pktgen_process_vlan(m, pid);       break;
		case UNKNOWN_PACKET:    /* FALL THRU */
		default: 				break;
		}
	} else {
		// Count the type of packets found.
		switch((int)pType) {
		case ETHER_TYPE_ARP:        info->stats.arp_pkts++;     break;
		case ETHER_TYPE_IPv4:       info->stats.ip_pkts++;      break;
		case ETHER_TYPE_IPv6:       info->stats.ipv6_pkts++;    break;
		case ETHER_TYPE_VLAN:       info->stats.vlan_pkts++;    break;
		default:			    	break;
		}
	}

    // Count the size of each packet.
    if ( plen == ETHER_MIN_LEN )
        info->sizes._64++;
    else if ( (plen >= (ETHER_MIN_LEN + 1)) && (plen <= 127) )
        info->sizes._65_127++;
    else if ( (plen >= 128) && (plen <= 255) )
        info->sizes._128_255++;
    else if ( (plen >= 256) && (plen <= 511) )
        info->sizes._256_511++;
    else if ( (plen >= 512) && (plen <= 1023) )
        info->sizes._512_1023++;
    else if ( (plen >= 1024) && (plen <= ETHER_MAX_LEN) )
        info->sizes._1024_1518++;
    else if ( plen < ETHER_MIN_LEN )
        info->sizes.runt++;
    else if ( plen >= (ETHER_MAX_LEN + 1) )
        info->sizes.jumbo++;

    // Process multicast and broadcast packets.
    if ( unlikely(((uint8_t *)m->pkt.data)[0] == 0xFF) ) {
		if ( (((uint64_t *)m->pkt.data)[0] & 0xFFFFFFFFFFFF0000LL) == 0xFFFFFFFFFFFF0000LL )
			info->sizes.broadcast++;
		else if ( ((uint8_t *)m->pkt.data)[0] & 1 )
			info->sizes.multicast++;
    }
}

/**************************************************************************//**
*
* pktgen_packet_classify_buld - Classify a set of packets in one call.
*
* DESCRIPTION
* Classify a list of packets and to improve classify performance.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

#define PREFETCH_OFFSET		3
static __inline__ void
pktgen_packet_classify_bulk(struct rte_mbuf ** pkts, int nb_rx, int pid )
{
	int j;

	/* Handle remaining prefetched packets */
	for (j = 0; j < nb_rx; j++)
		pktgen_packet_classify(pkts[j], pid);
}

/**************************************************************************//**
*
* pktgen_packet_dump - Dump the contents of a packet
*
* DESCRIPTION
* Dump the contents of a packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_packet_dump( struct rte_mbuf * m, int pid )
{
	port_info_t * info = &pktgen.info[pid];
	int plen = (m->pkt.pkt_len + FCS_SIZE);
	unsigned char *curr_data;
	struct rte_mbuf *curr_mbuf;

	/* Checking if info->dump_tail will not overflow is done in the caller */
	if (info->dump_list[info->dump_tail].data != NULL)
		rte_free(info->dump_list[info->dump_tail].data);

	info->dump_list[info->dump_tail].data = rte_malloc("Packet data", plen, 0);
	info->dump_list[info->dump_tail].len = plen;

	for (curr_data = info->dump_list[info->dump_tail].data, curr_mbuf = m;
		curr_mbuf != NULL;
		curr_data += curr_mbuf->pkt.data_len, curr_mbuf = curr_mbuf->pkt.next) {
		rte_memcpy(curr_data, curr_mbuf->pkt.data, curr_mbuf->pkt.data_len);
	}

	++info->dump_tail;
}

/**************************************************************************//**
*
* pktgen_packet_dump_bulk - Dump packet contents.
*
* DESCRIPTION
* Dump packet contents for later inspection.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_packet_dump_bulk(struct rte_mbuf ** pkts, int nb_dump, int pid )
{
	port_info_t * info = &pktgen.info[pid];
	int i;

	/* Don't dump more packets than the user asked */
	if (nb_dump > info->dump_count)
		nb_dump = info->dump_count;

	/* Don't overflow packet array */
	if (nb_dump > MAX_DUMP_PACKETS - info->dump_tail)
		nb_dump = MAX_DUMP_PACKETS - info->dump_tail;

	if (nb_dump == 0) {
		return;
	}

	for (i = 0; i < nb_dump; i++)
		pktgen_packet_dump(pkts[i], pid);

	info->dump_count -= nb_dump;
}

/**************************************************************************//**
*
* pktgen_packet_capture_bulk - Capture packets to memory.
*
* DESCRIPTION
* Capture packet contents to memory, so they can be written to disk later.
*
* A captured packet is stored as follows:
* - uint16_t: untruncated packet length
* - uint16_t: size of actual packet contents that are stored
* - unsigned char[]: packet contents (number of bytes stored equals previous
*       uint16_t)
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_packet_capture_bulk(struct rte_mbuf ** pkts, int nb_dump, capture_t *capture )
{
	unsigned int plen, i;
	struct rte_mbuf *pkt;
	unsigned char *buf_tail, *buf_end;

	buf_tail = (unsigned char *)capture->mz->addr + capture->mem_used;
	buf_end  = (unsigned char *)capture->mz->addr + capture->mz->len;

	/* Don't capture if buffer is full */
	if (buf_tail == buf_end)
		return;

	for (i = 0; i < nb_dump; i++) {
		pkt = pkts[i];
		/* If the packet is segmented by DPDK, only the contents of the first
		 * segment are captured. Capturing all segments uses too much CPU
		 * cycles, which causes packets to be dropped.
		 * Hence, data_len is used instead of pkt_len. */
		plen = pkt->pkt.data_len;

		/* If packet to capture is larger than available buffer size, stop
		 * capturing.
		 * The packet data is prepended by the untruncated packet length and
		 * the amount of captured data (which can be less than the packet size
		 * if DPDK has stored the packet contents in segmented mbufs).
		 */
		if (buf_tail + 2 * sizeof(uint16_t) + plen > buf_end) {
			/* If memory is available, write end-of-data sentinel. A packet
			 * length of 0 signals the end of the captured data to the routine
			 * that dumps the data to disk. */
			if (buf_end - buf_tail >= sizeof(uint16_t))
				*(uint16_t *)buf_tail = 0;

			capture->mem_used = capture->mz->len;
			return;
		}

		/* Write untruncated data length and size of the actually captured
		 * data. */
		*(uint16_t *)buf_tail = pkt->pkt.pkt_len;	/* untruncated length */
		buf_tail += sizeof(uint16_t);
		*(uint16_t *)buf_tail = plen;				/* captured data */
		buf_tail += sizeof(uint16_t);

		rte_memcpy(buf_tail, pkt->pkt.data, plen);
		buf_tail += plen;
	}

	/* Write end-of-data sentinel if memory is available. */
	if (buf_end - buf_tail >= sizeof(uint16_t))
		*(uint16_t *)buf_tail = 0;

	capture->mem_used = buf_tail - (unsigned char *)capture->mz->addr;
}

/**************************************************************************//**
*
* pktgen_send_arp - Send an ARP request packet.
*
* DESCRIPTION
* Create and send an ARP request packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_send_arp( uint32_t pid, uint32_t type, uint8_t seq_idx )
{
    port_info_t       * info = &pktgen.info[pid];
    pkt_seq_t         * pkt;
    struct rte_mbuf   * m ;
    struct ether_hdr  * eth;
    arpPkt_t          * arp;
    uint32_t            addr;
    uint8_t				qid = 0;

    pkt = &info->seq_pkt[seq_idx];
    m   = rte_pktmbuf_alloc(info->q[qid].special_mp);
    if ( unlikely(m == NULL) ) {
        scrn_fprintf(0,0, stdout, "%s: No packet buffers found\n", __FUNCTION__);
        return;
    }
    eth = rte_pktmbuf_mtod(m, struct ether_hdr *);
    arp = (arpPkt_t *)&eth[1];

    /* src and dest addr */
    memset(&eth->d_addr, 0xFF, 6);
    ether_addr_copy(&pkt->eth_src_addr, &eth->s_addr);
    eth->ether_type = htons(ETHER_TYPE_ARP);

    memset(arp, 0, sizeof(arpPkt_t));

    rte_memcpy( &arp->sha, &pkt->eth_src_addr, 6 );
    addr = htonl(pkt->ip_src_addr);
    inetAddrCopy(&arp->spa, &addr);

    if ( likely(type == GRATUITOUS_ARP) ) {
        rte_memcpy( &arp->tha, &pkt->eth_src_addr, 6 );
        addr = htonl(pkt->ip_src_addr);
        inetAddrCopy(&arp->tpa, &addr);
    } else {
        memset( &arp->tha, 0, 6 );
        addr = htonl(pkt->ip_dst_addr);
        inetAddrCopy(&arp->tpa, &addr);
    }

    /* Fill in the rest of the ARP packet header */
    arp->hrd    = htons(ETH_HW_TYPE);
    arp->pro    = htons(ETHER_TYPE_IPv4);
    arp->hln    = 6;
    arp->pln    = 4;
    arp->op     = htons(ARP_REQUEST);

    m->pkt.pkt_len  = 60;
    m->pkt.data_len = 60;

    pktgen_send_mbuf(m, pid, qid);

    pktgen_set_q_flags(info, qid, DO_TX_FLUSH);
}

/**************************************************************************//**
*
* pktgen_send_ping4 - Create and send a Ping or ICMP echo packet.
*
* DESCRIPTION
* Create a ICMP echo request packet and send the packet to a give port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_send_ping4( uint32_t pid, uint8_t seq_idx )
{
    port_info_t       * info = &pktgen.info[pid];
    pkt_seq_t         * ppkt = &info->seq_pkt[PING_PKT];
    pkt_seq_t         * spkt = &info->seq_pkt[seq_idx];
    struct rte_mbuf   * m ;
    uint8_t				qid = 0;

    m   = rte_pktmbuf_alloc(info->q[qid].special_mp);
    if ( unlikely(m == NULL) ) {
        scrn_fprintf(0,0,stdout,"%s: No packet buffers found\n", __FUNCTION__);
        return;
    }
	*ppkt = *spkt;		// Copy the sequence setup to the ping setup.
    pktgen_packet_ctor(info, PING_PKT, ICMP4_ECHO);
	rte_memcpy((uint8_t *)m->pkt.data, (uint8_t *)&ppkt->hdr, ppkt->pktSize);

    m->pkt.pkt_len  = ppkt->pktSize;
    m->pkt.data_len = ppkt->pktSize;

    pktgen_send_mbuf(m, pid, qid);

    pktgen_set_q_flags(info, qid, DO_TX_FLUSH);
}

#ifdef INCLUDE_PING6
/**************************************************************************//**
*
* pktgen_send_ping6 - Create and send a IPv6 ICMP echo packet.
*
* DESCRIPTION
* Create and send a IPv6 ICMP echo packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_send_ping6( uint32_t pid, uint32_t type, uint8_t seq_idx )
{
    port_info_t       * info = &pktgen.info[pid];
    pkt_seq_t         * pkt = &info->pkt[seq_idx];
    struct rte_mbuf   * m ;
    struct ether_hdr  * eth;
    arpPkt_t          * arp;
    uint32_t            addr;
    uint16_t			qid = 0;

    m   = rte_pktmbuf_alloc(info->q[qid].special_mp);
    if ( unlikely(m == NULL) ) {
        printf_info_info("%s: No packet buffers found\n", __FUNCTION__);
        return;
    }
    eth = rte_pktmbuf_mtod(m, struct ether_hdr *);
    arp = (arpPkt_t *)&eth[1];

    /* src and dest addr */
    memset(&eth->d_addr, 0xFF, 6);
    ether_addr_copy(&pkt->eth_src_addr, &eth->s_addr);
    eth->ether_type = htons(ETHER_TYPE_ARP);

    memset(arp, 0, sizeof(arpPkt_t));

    rte_memcpy( &arp->sha, &pkt->eth_src_addr, 6 );
    addr = htonl(pkt->ip_src_addr);
    inetAddrCopy(&arp->spa, &addr);

    if ( likely(type == GRATUITOUS_ARP) ) {
        rte_memcpy( &arp->tha, &pkt->eth_src_addr, 6 );
        addr = htonl(pkt->ip_src_addr);
        inetAddrCopy(&arp->tpa, &addr);
    } else {
        memset( &arp->tha, 0, 6 );
        addr = htonl(pkt->ip_dst_addr);
        inetAddrCopy(&arp->tpa, &addr);
    }

    /* Fill in the rest of the ARP packet header */
    arp->hrd    = htons(ETH_HW_TYPE);
    arp->pro    = htons(ETHER_TYPE_IPv4);
    arp->hln    = 6;
    arp->pln    = 4;
    arp->op     = htons(ARP_REQUEST);

    m->pkt.pkt_len  = 60;
    m->pkt.data_len = 60;

    pktgen_send_mbuf(m, pid, qid);

    pktgen_set_q_flags(info, qid, DO_TX_FLUSH);
}
#endif

/**************************************************************************//**
*
* pktgen_process_arp - Handle a ARP request input packet and send a response.
*
* DESCRIPTION
* Handle a ARP request input packet and send a response if required.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_process_arp( struct rte_mbuf * m, uint32_t pid, uint32_t vlan )
{
    port_info_t   * info = &pktgen.info[pid];
    pkt_seq_t     * pkt;
    uint32_t        i;
    struct ether_hdr *eth = rte_pktmbuf_mtod(m, struct ether_hdr *);
    arpPkt_t      * arp = (arpPkt_t *)&eth[1];

	/* Adjust for a vlan header if present */
	if ( vlan )
		arp = (arpPkt_t *)((char *)arp + sizeof(struct vlan_hdr));

    // Process all ARP requests if they are for us.
    if ( arp->op == htons(ARP_REQUEST) ) {
		if ((rte_atomic32_read(&info->port_flags) & PROCESS_GARP_PKTS) &&
 			(arp->tpa._32 == arp->spa._32) ) {		/* Must be a GARP packet */

			pkt = pktgen_find_matching_ipdst(info, arp->spa._32);

			/* Found a matching packet, replace the dst address */
			if ( pkt ) {
				rte_memcpy(&pkt->eth_dst_addr, &arp->sha, 6);
				pktgen_set_q_flags(info, wr_get_txque(pktgen.l2p, rte_lcore_id(), pid), DO_TX_CLEANUP);
				pktgen_redisplay(0);
			}
			return;
		}

		pkt = pktgen_find_matching_ipsrc(info, arp->tpa._32);

		/* ARP request not for this interface. */
		if ( likely(pkt != NULL) ) {
			/* Grab the source MAC address as the destination address for the port. */
			if ( unlikely(pktgen.flags & MAC_FROM_ARP_FLAG) ) {
				uint32_t    i;

				rte_memcpy(&pkt->eth_dst_addr, &arp->sha, 6);
				for (i = 0; i < info->seqCnt; i++)
					pktgen_packet_ctor(info, i, -1);
			}

			// Swap the two MAC addresses
			ethAddrSwap(&arp->sha, &arp->tha);

			// Swap the two IP addresses
			inetAddrSwap(&arp->tpa._32, &arp->spa._32);

			// Set the packet to ARP reply
			arp->op = htons(ARP_REPLY);

			// Swap the MAC addresses
			ethAddrSwap(&eth->d_addr, &eth->s_addr);

			// Copy in the MAC address for the reply.
			rte_memcpy(&arp->sha, &pkt->eth_src_addr, 6);
			rte_memcpy(&eth->s_addr, &pkt->eth_src_addr, 6);

			pktgen_send_mbuf(m, pid, 0);

			// Flush all of the packets in the queue.
			pktgen_set_q_flags(info, 0, DO_TX_FLUSH);

			// No need to free mbuf as it was reused
			return;
		}
	} else if ( arp->op == htons(ARP_REPLY) ) {
		pkt = pktgen_find_matching_ipsrc(info, arp->tpa._32);

		// ARP request not for this interface.
		if ( likely(pkt != NULL) ) {
			// Grab the real destination MAC address
			if ( pkt->ip_dst_addr == ntohl(arp->spa._32) )
				rte_memcpy(&pkt->eth_dst_addr, &arp->sha, 6);

			pktgen.flags |= PRINT_LABELS_FLAG;
		}
	}
}

/**************************************************************************//**
*
* pktgen_process_ping4 - Process a input ICMP echo packet for IPv4.
*
* DESCRIPTION
* Process a input packet for IPv4 ICMP echo request and send response if needed.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_process_ping4( struct rte_mbuf * m, uint32_t pid, uint32_t vlan )
{
    port_info_t   * info = &pktgen.info[pid];
    pkt_seq_t     * pkt;
    uint32_t        i;
    struct ether_hdr *eth = rte_pktmbuf_mtod(m, struct ether_hdr *);
    ipHdr_t       * ip = (ipHdr_t *)&eth[1];
    char            buff[24];

	/* Adjust for a vlan header if present */
	if ( vlan )
		ip = (ipHdr_t *)((char *)ip + sizeof(struct vlan_hdr));

    // Look for a ICMP echo requests, but only if enabled.
    if ( (rte_atomic32_read(&info->port_flags) & ICMP_ECHO_ENABLE_FLAG) &&
    		(ip->proto == PG_IPPROTO_ICMP) ) {
#if !defined(RTE_ARCH_X86_64)
        icmpv4Hdr_t * icmp = (icmpv4Hdr_t *)((uint32_t)ip + sizeof(ipHdr_t));
#else
        icmpv4Hdr_t * icmp = (icmpv4Hdr_t *)((uint64_t)ip + sizeof(ipHdr_t));
#endif

        // We do not handle IP options, which will effect the IP header size.
        if ( unlikely(cksum(icmp, (m->pkt.data_len - sizeof(struct ether_hdr) - sizeof(ipHdr_t)), 0)) ) {
            printf_info("ICMP checksum failed\n");
            return;
        }

        if ( unlikely(icmp->type == ICMP4_ECHO) ) {
            if ( ntohl(ip->dst) == INADDR_BROADCAST ) {
                printf_info("IP address %s is a Broadcast\n",
                        inet_ntop4(buff, sizeof(buff), ip->dst, INADDR_BROADCAST));
                return;
            }

            // Toss all broadcast addresses and requests not for this port
            pkt = pktgen_find_matching_ipsrc(info, ip->dst);

            // ARP request not for this interface.
            if ( unlikely(pkt == NULL) ) {
                printf_info("IP address %s not found\n",
                        inet_ntop4(buff, sizeof(buff), ip->dst, INADDR_BROADCAST));
                return;
            }

            info->stats.echo_pkts++;

            icmp->type  = ICMP4_ECHO_REPLY;

            /* Recompute the ICMP checksum */
            icmp->cksum = 0;
            icmp->cksum = cksum(icmp, (m->pkt.data_len - sizeof(struct ether_hdr) - sizeof(ipHdr_t)), 0);

            // Swap the IP addresses.
            inetAddrSwap(&ip->src, &ip->dst);

            // Bump the ident value
            ip->ident   = htons(ntohs(ip->ident) + m->pkt.data_len);

            // Recompute the IP checksum
            ip->cksum   = 0;
            ip->cksum   = cksum(ip, sizeof(ipHdr_t), 0);

            // Swap the MAC addresses
            ethAddrSwap(&eth->d_addr, &eth->s_addr);

            pktgen_send_mbuf(m, pid, 0);

            pktgen_set_q_flags(info, 0, DO_TX_FLUSH);

            // No need to free mbuf as it was reused.
            return;
        } else if ( unlikely(icmp->type == ICMP4_ECHO_REPLY) ) {
            info->stats.echo_pkts++;
        }
    }
}

/**************************************************************************//**
*
* pktgen_process_ping6 - Process a IPv6 ICMP echo request packet.
*
* DESCRIPTION
* Process a IPv6 ICMP echo request packet and send response if needed.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_process_ping6( struct rte_mbuf * m, uint32_t pid, uint32_t vlan )
{
#if 0 /* Broken needs to be updated to do IPv6 packets */
    port_info_t     * info = &pktgen.info[pid];
    struct ether_hdr *eth = rte_pktmbuf_mtod(m, struct ether_hdr *);
    ipv6Hdr_t       * ip = (ipv6Hdr_t *)&eth[1];

	/* Adjust for a vlan header if present */
	if ( vlan )
		ip = (ipv6Hdr_t *)((char *)ip + sizeof(struct vlan_hdr));

    // Look for a ICMP echo requests, but only if enabled.
    if ( (rte_atomic32_read(&info->port_flags) & ICMP_ECHO_ENABLE_FLAG) &&
    		(ip->next_header == PG_IPPROTO_ICMPV6) ) {
#if !defined(RTE_ARCH_X86_64)
        icmpv4Hdr_t * icmp = (icmpv4Hdr_t *)((uint32_t)ip + sizeof(ipHdr_t));
#else
        icmpv4Hdr_t * icmp = (icmpv4Hdr_t *)((uint64_t)ip + sizeof(ipHdr_t));
#endif
        // We do not handle IP options, which will effect the IP header size.
        if ( cksum(icmp, (m->pkt.data_len - sizeof(struct ether_hdr) - sizeof(ipHdr_t)), 0) ) {
            printf_info("ICMP checksum failed\n");
            goto leave:
        }

        if ( icmp->type == ICMP4_ECHO ) {
            // Toss all broadcast addresses and requests not for this port
            if ( (ip->dst == INADDR_BROADCAST) || (ip->dst != info->ip_src_addr) ) {
                char        buff[24];
                printf_info("IP address %s != ",
                        inet_ntop4(buff, sizeof(buff), ip->dst, INADDR_BROADCAST));
                printf_info("%s\n",
                        inet_ntop4(buff, sizeof(buff), htonl(info->ip_src_addr), INADDR_BROADCAST));
                goto leave;
            }

            info->echo_pkts++;

            icmp->type  = ICMP4_ECHO_REPLY;

            /* Recompute the ICMP checksum */
            icmp->cksum = 0;
            icmp->cksum = cksum(icmp, (m->pkt.data_len - sizeof(struct ether_hdr) - sizeof(ipHdr_t)), 0);

            // Swap the IP addresses.
            inetAddrSwap(&ip->src, &ip->dst);

            // Bump the ident value
            ip->ident   = htons(ntohs(ip->ident) + m->pkt.data_len);

            // Recompute the IP checksum
            ip->cksum   = 0;
            ip->cksum   = cksum(ip, sizeof(ipHdr_t), 0);

            // Swap the MAC addresses
            ethAddrSwap(&eth->d_addr, &eth->s_addr);

            pktgen_send_mbuf(m, pid, 0);

            pktgen_set_q_flags(info, 0, DO_TX_FLUSH);

            // No need to free mbuf as it was reused
            return;
        }
    }
leave:
#else
#endif
}

/**************************************************************************//**
*
* pktgen_process_vlan - Process a VLAN packet
*
* DESCRIPTION
* Process a input VLAN packet.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_process_vlan( struct rte_mbuf * m, uint32_t pid )
{
	pktType_e        pType;
    struct ether_hdr *eth;
    struct vlan_hdr  *vlan_hdr;
    port_info_t      *info = &pktgen.info[pid];

    eth = rte_pktmbuf_mtod(m, struct ether_hdr *);

    /* Now dealing with the inner header */
    vlan_hdr = (struct vlan_hdr*)(eth+1);

    pType = ntohs(vlan_hdr->eth_proto);

	/* No support for nested tunnel */
	switch((int)pType) {
	case ETHER_TYPE_ARP:    info->stats.arp_pkts++;         pktgen_process_arp(m, pid, 1);		break;
	case ETHER_TYPE_IPv4:   info->stats.ip_pkts++;          pktgen_process_ping4(m, pid, 1);    break;
	case ETHER_TYPE_IPv6:   info->stats.ipv6_pkts++;        pktgen_process_ping6(m, pid, 1);    break;
	case UNKNOWN_PACKET:    /* FALL THRU */
	default:                break;
	};
}

/**************************************************************************//**
*
* pktgen_send_special - Send a special packet to the given port.
*
* DESCRIPTION
* Create a special packet in the buffer provided.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_send_special(port_info_t * info)
{
    uint32_t    flags = rte_atomic32_read(&info->port_flags);
    uint32_t    s;

    if ( unlikely((flags & SEND_SPECIAL_REQUEST) == 0) )
        return;

    pktgen_clr_port_flags(info, SEND_SPECIAL_REQUEST);

    for(s=0; s < info->seqCnt; s++) {
        if ( unlikely(flags & SEND_GRATUITOUS_ARP) )
            pktgen_send_arp(info->pid, GRATUITOUS_ARP, s);
        if ( likely(flags & SEND_ARP_REQUEST) )
            pktgen_send_arp(info->pid, 0, s);

        if ( likely(flags & SEND_PING4_REQUEST) )
            pktgen_send_ping4(info->pid, s);
#ifdef INCLUDE_PING6
        if ( flags & SEND_PING6_REQUEST )
            pktgen_send_ping6(info->pid, s);
#endif
    }

	if ( unlikely(flags & SEND_GRATUITOUS_ARP) )
		pktgen_send_arp(info->pid, GRATUITOUS_ARP, SINGLE_PKT);
	if ( likely(flags & SEND_ARP_REQUEST) )
		pktgen_send_arp(info->pid, 0, SINGLE_PKT);

	if ( likely(flags & SEND_PING4_REQUEST) )
		pktgen_send_ping4(info->pid, SINGLE_PKT);
#ifdef INCLUDE_PING6
	if ( flags & SEND_PING6_REQUEST )
		pktgen_send_ping6(info->pid, SINGLE_PKT);
#endif
}

/**************************************************************************//**
*
* pktgen_range_ctor - Construct a range packet in buffer provided.
*
* DESCRIPTION
* Build the special range packet in the buffer provided.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_range_ctor(port_info_t * info, pkt_seq_t * pkt)
{
	range_info_t *  range = &info->range;

	switch(pkt->ethType) {
	case ETHER_TYPE_IPv4:
		switch(pkt->ipProto) {
		case PG_IPPROTO_TCP:
		case PG_IPPROTO_UDP:
			if ( unlikely(range->src_port_inc != 0) ) {
				uint16_t sport = pkt->sport;
				sport += range->src_port_inc;
				if ( sport < range->src_port_min )
					sport = range->src_port_max;
				if ( sport > range->src_port_max )
					sport = range->src_port_min;
				pkt->sport = sport;
			}
			else
				pkt->sport = range->src_port;

			if ( unlikely(range->dst_port_inc != 0) ) {
				uint16_t dport = pkt->dport;
				dport += range->dst_port_inc;
				if ( dport < range->dst_port_min )
					dport = range->dst_port_max;
				if ( dport > range->dst_port_max )
					dport = range->dst_port_min;
				pkt->dport = dport;
			}
			else
				pkt->dport = range->dst_port;

            if (unlikely(range->src_ip_inc != 0)) {
                uint32_t p = pkt->ip_src_addr;
                p += range->src_ip_inc;
                if (p < range->src_ip_min)
                    p = range->src_ip_max;
                else if (p > range->src_ip_max)
                    p = range->src_ip_min;
                pkt->ip_src_addr = p;
            }
			else 
				pkt->ip_src_addr = range->src_ip;

            if (unlikely(range->dst_ip_inc != 0)) {
                uint32_t p = pkt->ip_dst_addr;
                p += range->dst_ip_inc;
                if (p < range->dst_ip_min)
                    p = range->dst_ip_max;
                else if (p > range->dst_ip_max)
                    p = range->dst_ip_min;
                pkt->ip_dst_addr = p;
            }
			else 
				pkt->ip_dst_addr = range->dst_ip;

            if (unlikely(range->vlan_id_inc != 0)) {
                uint32_t p = pkt->vlanid;
                p += range->vlan_id_inc;
                if (p < range->vlan_id_min)
                    p = range->vlan_id_max;
                else if (p > range->vlan_id_max)
                    p = range->vlan_id_min;
                pkt->vlanid = p;
            }
			else
				pkt->vlanid = range->vlan_id;

            if (unlikely(range->pkt_size_inc != 0)) {
                uint32_t p = pkt->pktSize;
                p += range->pkt_size_inc;
                if (p < range->pkt_size_min)
                    p = range->pkt_size_max;
                else if (p > range->pkt_size_max)
                    p = range->pkt_size_min;
                pkt->pktSize = p;
            }
			else
				pkt->pktSize = range->pkt_size;

			if (unlikely(range->src_mac_inc != 0)) {
				uint64_t p;

				inet_mtoh64(&pkt->eth_src_addr, &p);

				p += range->src_mac_inc;
				if (p < range->src_mac_min)
					p = range->src_mac_max;
				else if (p > range->src_mac_max)
					p = range->src_mac_min;

				inet_h64tom(p, &pkt->eth_src_addr);
			}
			else
				inet_h64tom(range->src_mac, &pkt->eth_src_addr);

			if (unlikely(range->dst_mac_inc != 0)) {
				uint64_t p;

				inet_mtoh64(&pkt->eth_dst_addr, &p);

				p += range->dst_mac_inc;
				if (p < range->dst_mac_min)
					p = range->dst_mac_max;
				else if (p > range->dst_mac_max)
					p = range->dst_mac_min;

				inet_h64tom(p, &pkt->eth_dst_addr);
			}
			else
				inet_h64tom(range->dst_mac, &pkt->eth_dst_addr);

			break;
		default:
			printf_info("%s: IPv4 ipProto %02x\n", __FUNCTION__, pkt->ipProto);
			break;
		}
		break;
	case ETHER_TYPE_IPv6:
		switch(pkt->ipProto) {
		case PG_IPPROTO_UDP:
		case PG_IPPROTO_TCP:
			// TODO: Need to handle the IPv6 packets.
			break;
		default:
			printf_info("%s: IPv6 ipProto %04x\n", __FUNCTION__, pkt->ipProto);
			break;
		}
		break;
	default:
		printf_info("%s: ethType %04x\n", __FUNCTION__, pkt->ethType);
		break;
	}
}

/**************************************************************************//**
*
* pktgen_setup_packets - Setup the default packets to be sent.
*
* DESCRIPTION
* Construct the default set of packets for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_setup_packets(port_info_t * info, struct rte_mempool * mp, uint8_t qid)
{
	struct rte_mbuf	* m, * mm;
	pkt_seq_t * pkt;

	pktgen_clr_q_flags(info, qid, CLEAR_FAST_ALLOC_FLAG);

	if ( mp == info->q[qid].pcap_mp )
		return;

	mm	= NULL;
	pkt = NULL;

	if ( mp == info->q[qid].tx_mp )
		pkt = &info->seq_pkt[SINGLE_PKT];
	else if ( mp == info->q[qid].range_mp )
		pkt = &info->seq_pkt[RANGE_PKT];
	else if ( mp == info->q[qid].seq_mp )
		pkt = &info->seq_pkt[info->seqIdx];

	// allocate each mbuf and put them on a list to be freed.
	for(;;) {
		m = rte_pktmbuf_alloc_noreset(mp);
		if ( unlikely(m == NULL) )
			break;

		// Put the allocated mbuf into a list to be freed later
		m->pkt.next = mm;
		mm = m;

		if ( mp == info->q[qid].tx_mp ) {
			pktgen_packet_ctor(info, SINGLE_PKT, -1);

			rte_memcpy((uint8_t *)m->pkt.data, (uint8_t *)&pkt->hdr, MAX_PKT_SIZE);

			m->pkt.pkt_len  = pkt->pktSize;
			m->pkt.data_len = pkt->pktSize;
		} else if ( mp == info->q[qid].range_mp ) {
			pktgen_range_ctor(info, pkt);
			pktgen_packet_ctor(info, RANGE_PKT, -1);

			rte_memcpy((uint8_t *)m->pkt.data, (uint8_t *)&pkt->hdr, MAX_PKT_SIZE);

			m->pkt.pkt_len  = pkt->pktSize;
			m->pkt.data_len = pkt->pktSize;
		} else if ( mp == info->q[qid].seq_mp ) {
			pktgen_packet_ctor(info, info->seqIdx++, -1);
			if ( unlikely(info->seqIdx >= info->seqCnt) )
				info->seqIdx = 0;

			rte_memcpy((uint8_t *)m->pkt.data, (uint8_t *)&pkt->hdr, MAX_PKT_SIZE);

			m->pkt.pkt_len  = pkt->pktSize;
			m->pkt.data_len = pkt->pktSize;

			// move to the next packet in the sequence.
			pkt = &info->seq_pkt[info->seqIdx];
		}
	}

	// Free all of the mbufs
	if ( likely(mm != 0) ) {
		while( (m = mm) != NULL ) {
			mm = m->pkt.next;
			m->pkt.next = NULL;
			rte_pktmbuf_free(m);
		}
	}
}

/**************************************************************************//**
*
* pktgen_send_pkts - Send a set of packet buffers to a given port.
*
* DESCRIPTION
* Transmit a set of packets mbufs to a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_send_pkts(port_info_t * info, uint8_t qid, struct rte_mempool * mp)
{
	int			txCnt;

	if ( unlikely(rte_atomic32_read(&info->q[qid].flags) & CLEAR_FAST_ALLOC_FLAG) )
		pktgen_setup_packets(info, mp, qid);

	txCnt = info->current_tx_count;
	if ( likely(txCnt == 0) || unlikely(txCnt > info->tx_burst) )
		txCnt = info->tx_burst;

	info->q[qid].tx_mbufs.len = rte_pktmbuf_alloc_bulk_noreset(mp, (void **)info->q[qid].tx_mbufs.m_table, txCnt);
	if ( likely(info->q[qid].tx_mbufs.len) )
		pktgen_send_burst(info, qid);

	if ( unlikely(info->current_tx_count) ) {
		info->current_tx_count -= txCnt;
        if ( unlikely(info->current_tx_count == 0) ) {
            info->transmitting = 0;
            pktgen_set_q_flags(info, qid, DO_TX_CLEANUP);
        }
	}
}

/**************************************************************************//**
*
* pktgen_main_transmit - Determine the next packet format to transmit.
*
* DESCRIPTION
* Determine the next packet format to transmit for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_main_transmit(port_info_t * info, uint8_t qid)
{
    uint32_t		flags;

	flags = rte_atomic32_read(&info->port_flags);

	/*
	 * Transmit special packets if enabled
	 */
	if ( unlikely(flags & SEND_SPECIAL_REQUEST) )
		pktgen_send_special(info);

	// When not transmitting on this port then continue.
	if ( unlikely(info->transmitting == 0) )
		info->current_tx_count 	= 0;
	else {
		struct rte_mempool * mp	= info->q[qid].tx_mp;

		if ( unlikely(flags & (SEND_RANGE_PKTS|SEND_PCAP_PKTS|SEND_SEQ_PKTS)) ) {
			if ( flags & SEND_RANGE_PKTS )
				mp = info->q[qid].range_mp;
			else if ( flags & SEND_SEQ_PKTS )
				mp = info->q[qid].seq_mp;
			else if ( flags & SEND_PCAP_PKTS )
				mp = info->q[qid].pcap_mp;
		}

		pktgen_send_pkts(info, qid, mp);
	}

	flags = rte_atomic32_read(&info->q[qid].flags);
	if ( unlikely(flags & (DO_TX_CLEANUP |  DO_TX_FLUSH)) ) {
		if ( flags & DO_TX_CLEANUP )
			pktgen_tx_cleanup(info, qid);
		else if ( flags & DO_TX_FLUSH )
			pktgen_tx_flush(info, qid);
	}
}

/**************************************************************************//**
*
* pktgen_main_receive - Main receive routine for packets of a port.
*
* DESCRIPTION
* Handle the main receive set of packets on a given port plus handle all of the
* input processing if required.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
pktgen_main_receive(port_info_t * info, uint8_t lid, uint8_t idx, struct rte_mbuf *pkts_burst[])
{
	uint32_t nb_rx, pid, qid;
	capture_t *capture;
	int i;

	pid = info->pid;
	qid = wr_get_rxque(pktgen.l2p, lid, idx);

	/*
	 * Read packet from RX queues and free the mbufs
	 */
	if ( (nb_rx = rte_eth_rx_burst(pid, qid, pkts_burst, info->tx_burst)) == 0 )
		return;

	/* Prefetch packets */
	for (i = 0; i < nb_rx; i++)
		rte_prefetch0(rte_pktmbuf_mtod(pkts_burst[i], void *));

	pktgen_packet_classify_bulk(pkts_burst, nb_rx, pid);

	if ( unlikely(info->dump_count > 0) )
		pktgen_packet_dump_bulk(pkts_burst, nb_rx, pid);

	if ( unlikely(rte_atomic32_read(&info->port_flags) & CAPTURE_PKTS) ) {
		capture = &pktgen.capture[pktgen.core_info[lid].s.socket_id];
		if ( unlikely((capture->port == pid) &&
				   (capture->lcore == lid)) ) {
			pktgen_packet_capture_bulk(pkts_burst, nb_rx, capture);
		}
	}

	for (i = 0; i < nb_rx; i++)
		rte_pktmbuf_free(pkts_burst[i]);
}

/**************************************************************************//**
*
* pktgen_main_rxtx_loop - Single thread loop for tx/rx packets
*
* DESCRIPTION
* Handle sending and receiving packets from a given set of ports. This is the
* main loop or thread started on a single core.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_main_rxtx_loop(uint8_t lid)
{
    struct rte_mbuf *pkts_burst[DEFAULT_PKT_BURST];
	port_info_t	  * infos[RTE_MAX_ETHPORTS];
	uint8_t		 	qids[RTE_MAX_ETHPORTS];
    uint8_t			idx, pid, txcnt, rxcnt;
    uint64_t		 curr_tsc;
	uint64_t		tx_next_cycle;		/**< Next cycle to send a burst of traffic */

    txcnt	= wr_get_lcore_txcnt(pktgen.l2p, lid);
	rxcnt	= wr_get_lcore_rxcnt(pktgen.l2p, lid);
    printf_info("=== RX/TX processing on lcore %2d, rxcnt %d, txcnt %d, port/qid, ",
    		lid, rxcnt, txcnt);

	for( idx = 0; idx < wr_get_lcore_txcnt(pktgen.l2p, lid); idx++ ) {
		pid = wr_get_rx_pid(pktgen.l2p, lid, idx);
    	if ( (infos[idx] = wr_get_port_private(pktgen.l2p, pid)) == NULL )
    		continue;
    	qids[idx] = wr_get_txque(pktgen.l2p, lid , pid);
    	printf_info("%d/%d ", infos[idx]->pid, qids[idx]);
	}
	printf_info("\n");

    tx_next_cycle	= 0;

    wr_start_lcore(pktgen.l2p, lid);
    do {
		for(idx = 0; idx < rxcnt; idx++) {
			/*
			 * Read packet from RX queues and free the mbufs
			 */
			pktgen_main_receive(infos[idx], lid, idx, pkts_burst);
		}

        curr_tsc = rte_rdtsc();

		// Determine when is the next time to send packets
		if ( unlikely(curr_tsc >= tx_next_cycle) ) {

			tx_next_cycle = curr_tsc + infos[0]->tx_cycles;

	    	for(idx = 0; idx < txcnt; idx++) {
				/*
				 * Transmit packets at a given rate.
				 */
				pktgen_main_transmit(infos[idx], qids[idx]);
	    	}
		}

		// Exit loop when flag is set.
    } while ( wr_lcore_is_running(pktgen.l2p, lid) );

    dbgPrintf("%s: Exit %d\n", __FUNCTION__, lid);
	pktgen_cleanup(lid);
}

/**************************************************************************//**
*
* pktgen_main_tx_loop - Main transmit loop for a core, no receive packet handling
*
* DESCRIPTION
* When Tx and Rx are split across two cores this routing handles the tx packets.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_main_tx_loop(uint8_t lid)
{
    uint8_t		  idx, txcnt, pid;
    port_info_t * infos[RTE_MAX_ETHPORTS];
    uint8_t       qids[RTE_MAX_ETHPORTS];
    uint64_t	  curr_tsc;
	uint64_t	  tx_next_cycle;		/**< Next cycle to send a burst of traffic */

    txcnt = wr_get_lcore_txcnt(pktgen.l2p, lid);
    printf_info("=== TX processing on lcore %2d, txcnt %d, port/qid, ", lid, txcnt);

	for( idx = 0; idx < txcnt; idx++ ) {
		pid = wr_get_tx_pid(pktgen.l2p, lid, idx);
    	if ( (infos[idx] = wr_get_port_private(pktgen.l2p, pid)) == NULL )
    		continue;
    	qids[idx] = wr_get_txque(pktgen.l2p, lid, pid);
    	printf_info("%d/%d ", infos[idx]->pid, qids[idx]);
	}
	printf_info("\n");

	tx_next_cycle = 0;

	wr_start_lcore(pktgen.l2p, lid);
    do {
		curr_tsc = rte_rdtsc();

		// Determine when is the next time to send packets
		if ( unlikely(curr_tsc >= tx_next_cycle) ) {

			tx_next_cycle = curr_tsc + infos[0]->tx_cycles;

	    	for(idx = 0; idx < txcnt; idx++) {
				/* Transmit packets */
				pktgen_main_transmit(infos[idx], qids[idx]);
	    	}
    	}

		// Exit loop when flag is set.
    } while( wr_lcore_is_running(pktgen.l2p, lid) );

    dbgPrintf("%s: Exit %d\n", __FUNCTION__, lid);

	pktgen_cleanup(lid);
}

/**************************************************************************//**
*
* pktgen_main_rx_loop - Handle only the rx packets for a set of ports.
*
* DESCRIPTION
* When Tx and Rx processing is split between two ports this routine handles
* only the receive packets.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_main_rx_loop(uint8_t lid)
{
    struct rte_mbuf *pkts_burst[DEFAULT_PKT_BURST];
	uint8_t			pid, idx, rxcnt;
	port_info_t	  * infos[RTE_MAX_ETHPORTS];

	rxcnt = wr_get_lcore_rxcnt(pktgen.l2p, lid);
    printf_info("=== RX processing on lcore %2d, rxcnt %d, port/qid, ",
    		lid, rxcnt);

    memset(infos, '\0', sizeof(infos));
	for( idx = 0; idx < rxcnt; idx++ ) {
		pid = wr_get_rx_pid(pktgen.l2p, lid, idx);
    	if ( (infos[idx] = wr_get_port_private(pktgen.l2p, pid)) == NULL )
    		continue;
    	printf_info("%d/%d ", infos[idx]->pid, wr_get_rxque(pktgen.l2p, lid, pid));
	}
	printf_info("\n");

	wr_start_lcore(pktgen.l2p, lid);
    do {
		for(idx = 0; idx < rxcnt; idx++) {
			// Read packet from RX queues and free the mbufs
			pktgen_main_receive(infos[idx], lid, idx, pkts_burst);
		}
		// Exit loop when flag is set.
    } while( wr_lcore_is_running(pktgen.l2p, lid) );

    dbgPrintf("%s: Exit %d\n", __FUNCTION__, lid);
    pktgen_cleanup(lid);
}

/**************************************************************************//**
*
* pktgen_launch_one_lcore - Launch a single logical core thread.
*
* DESCRIPTION
* Help launch a single thread on one logical core.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static int
pktgen_launch_one_lcore(__attribute__ ((unused)) void * arg)
{
	uint8_t		lid = rte_lcore_id();

	if ( pktgen_has_work() )
		return 0;

	rte_delay_ms((lid + 1) *21);

	switch(wr_get_type(pktgen.l2p, lid)) {
	case RX_TYPE:				pktgen_main_rx_loop(lid);		break;
	case TX_TYPE:				pktgen_main_tx_loop(lid);		break;
	case (RX_TYPE | TX_TYPE):	pktgen_main_rxtx_loop(lid); 	break;
	}
	return 0;
}

/**************************************************************************//**
*
* display_topline - Print out the top line on the screen.
*
* DESCRIPTION
* Print out the top line on the screen and any other information.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
display_topline(const char * msg)
{
	scrn_center(1, "%s  %s, %s", msg, wr_copyright_msg(), wr_powered_by());
}

/**************************************************************************//**
*
* display_dashline - Print out the dashed line on the screen.
*
* DESCRIPTION
* Print out the dashed line on the screen and any other information.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static __inline__ void
display_dashline(int last_row)
{
	int		i;

	scrn_setw(last_row);
	last_row--;
    scrn_pos(last_row, 1);
    for(i=0; i<(pktgen.scrn->ncols-15); i++)
    	printf_info("-");
    scrn_printf(last_row, 3, " Pktgen %s ", pktgen_version());
}

/**************************************************************************//**
*
* pktgen_print_static_data - Display the static data on the screen.
*
* DESCRIPTION
* Display a set of port static data on the screen.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_print_static_data(void)
{
    port_info_t * info;
    uint32_t pid, col, row, sp, ip_row;
    pkt_seq_t * pkt;
    char buff[32];

    display_topline("** Main Page **");

    scrn_printf(1, 3, "Ports %d-%d of %d", pktgen.starting_port, (pktgen.ending_port - 1), pktgen.nb_ports);

    row = PORT_STATE_ROW;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  Flags:Port");

    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Link State");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Pkts/s  Rx");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "        Tx");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "MBits/s Rx/Tx");

    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Broadcast");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Multicast");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  64 Bytes");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  65-127");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  128-255");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  256-511");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  512-1023");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "  1024-1518");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Runts/Jumbos");

    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Errors Rx/Tx");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Total Rx Pkts");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "      Tx Pkts");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "      Rx MBs ");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "      Tx MBs ");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "ARP/ICMP Pkts");
	if ( pktgen.flags & TX_DEBUG_FLAG ) {
		scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Tx Overrun");
		scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Cycles per Tx");
	}

    ip_row = ++row;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Tx Count/% Rate");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "PktSize/Tx Burst");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Src/Dest Port");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Pkt Type:VLAN ID");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Dst  IP Address");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Src  IP Address");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Dst MAC Address");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Src MAC Address");

    // Get the last location to use for the window starting row.
    pktgen.last_row = ++row;
    display_dashline(pktgen.last_row);

    // Display the colon after the row label.
    for(row = PORT_STATE_ROW; row < ((ip_row + IP_ADDR_ROWS) - 2); row++)
        scrn_printf(row, COLUMN_WIDTH_0-1, ":");

    sp = pktgen.starting_port;
    for (pid = 0; pid < pktgen.nb_ports_per_page; pid++) {
        if ( wr_get_map(pktgen.l2p, pid+sp, RTE_MAX_LCORE) == 0 )
            continue;

        info	= &pktgen.info[pid+sp];

        pkt		= &info->seq_pkt[SINGLE_PKT];

        // Display Port information Src/Dest IP addr, Netmask, Src/Dst MAC addr
        col = (COLUMN_WIDTH_1 * pid) + COLUMN_WIDTH_0;
        row = ip_row;

        pktgen_transmit_count_rate(pid, buff, sizeof(buff));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

        snprintf(buff, sizeof(buff), "%d/%d", pkt->pktSize + FCS_SIZE, info->tx_burst);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);
        snprintf(buff, sizeof(buff), "%d/%d", pkt->sport, pkt->dport);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);
        snprintf(buff, sizeof(buff), "%s/%s:%04x", (pkt->ethType == ETHER_TYPE_IPv4)? "IPv4" :
                                              (pkt->ethType == ETHER_TYPE_IPv6)? "IPv6" : "Other",
                                              (pkt->ipProto == PG_IPPROTO_TCP)? "TCP" :
                                              (pkt->ipProto == PG_IPPROTO_ICMP)? "ICMP" : "UDP",
                                               pkt->vlanid);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(pkt->ip_dst_addr), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(pkt->ip_src_addr), pkt->ip_mask));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &pkt->eth_dst_addr));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &pkt->eth_src_addr));
    }

    // Display the string for total pkts/s rate of all ports
    col = (COLUMN_WIDTH_1 * pktgen.nb_ports_per_page) + COLUMN_WIDTH_0;
    scrn_printf(LINK_STATE_ROW, col, "%*s", COLUMN_WIDTH_1, "---TotalRate---"); scrn_eol();

    pktgen.flags &= ~PRINT_LABELS_FLAG;
}

/**************************************************************************//**
*
* pktgen_print_pcap - Display the pcap data page.
*
* DESCRIPTION
* Display the pcap data page on the screen.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_print_pcap(uint16_t pid)
{
    uint32_t    i, row, col, max_pkts, len;
    uint16_t	type, vlan, skip;
    uint8_t		proto;
    port_info_t * info;
    pkt_hdr_t   * hdr;
    pcap_info_t * pcap;
    pcaprec_hdr_t    pcap_hdr;
    char buff[64];
    char pkt_buff[2048];

    display_topline("** PCAP Page **");
    scrn_printf(1, 3, "Port %d of %d", pid, pktgen.nb_ports);

    info = &pktgen.info[pid];
    pcap = info->pcap;

    row = PORT_STATE_ROW;
    col = 1;
    if ( pcap == NULL ) {
    	scrn_center(10, "** Port does not have a PCAP file assigned **");
    	row = 28;
    	goto leave;
    }

    scrn_eol_pos(row, col);
    scrn_printf(row++, col, "Port: %d, PCAP Count: %d of %d",
    		pid, pcap->pkt_idx, pcap->pkt_count);
    scrn_printf(row++, col, "%*s %*s%*s%*s%*s%*s%*s%*s",
            5, "Seq",
            COLUMN_WIDTH_0, "Dst MAC",
            COLUMN_WIDTH_0, "Src MAC",
            COLUMN_WIDTH_0, "Dst IP",
            COLUMN_WIDTH_0+2, "Src IP",
            12, "Port S/D",
            15, "Protocol:VLAN",
            9, "Size-FCS");

    max_pkts = pcap->pkt_idx + PCAP_PAGE_SIZE;
    if ( max_pkts > pcap->pkt_count )
    	max_pkts = pcap->pkt_count;

    wr_pcap_skip(pcap, pcap->pkt_idx);

    for(i = pcap->pkt_idx; i < max_pkts; i++) {
        col = 1;
        skip = 0;

        len = wr_pcap_read(pcap, &pcap_hdr, pkt_buff, sizeof(pkt_buff));
        if ( len == 0 )
        	break;

    	// Skip any jumbo packets larger then buffer.
        if ( pcap_hdr.incl_len > sizeof(pkt_buff) ) {
    		i--;
    		skip++;
        }
    	// Skip packets that are not normal IP packets.
    	type = ntohs( ((uint16_t *)pkt_buff)[6] );
    	if ( unlikely(type == ETHER_TYPE_VLAN) )
    		type = ntohs( ((uint16_t *)pkt_buff)[8] );

        if ( unlikely(type < MAX_ETHER_TYPE_SIZE) )
            skip++;

        hdr = (pkt_hdr_t *)&pkt_buff[0];

       	scrn_eol_pos(row, col);

        scrn_printf(row, col, "%5d:", i);
        col += 7;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &hdr->eth.d_addr));
        col += COLUMN_WIDTH_1;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &hdr->eth.s_addr));
        col += COLUMN_WIDTH_1;

        type = ntohs(hdr->eth.ether_type);
        proto = hdr->u.ipv4.proto;
        vlan = 0;
        if ( type == ETHER_TYPE_VLAN ) {
        	vlan = ntohs( ((uint16_t *)&hdr->eth.ether_type)[1] );
        	type = ntohs( ((uint16_t *)&hdr->eth.ether_type)[2] );
        	proto = ((ipHdr_t *)((char *)&hdr->u.ipv4 + 4))->proto;
        }

        if ( (type == ETHER_TYPE_IPv4) ) {
			scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), hdr->u.ipv4.dst, 0xFFFFFFFF));
			col += COLUMN_WIDTH_1;
			scrn_printf(row, col, "%*s", COLUMN_WIDTH_1+2, inet_ntop4(buff, sizeof(buff), hdr->u.ipv4.src, 0xFFFFFFFF));
			col += COLUMN_WIDTH_1+2;

	        snprintf(buff, sizeof(buff), "%d/%d", ntohs(hdr->u.uip.udp.sport), ntohs(hdr->u.uip.udp.dport));
	        scrn_printf(row, col, "%*s", 12, buff);
	        col += 12;
        } else {
        	skip++;
        	col += ((2 * COLUMN_WIDTH_1) + 2 + 12);
        }
        snprintf(buff, sizeof(buff), "%s/%s:%4d", (type == ETHER_TYPE_IPv4)? "IPv4" :
                                                   (type == ETHER_TYPE_IPv6)? "IPv6" : "Other",
                                                   (type == PG_IPPROTO_TCP)? "TCP" :
                                                   (proto == PG_IPPROTO_ICMP)? "ICMP" : "UDP",
                                                   (vlan & 0xFFF));
        scrn_printf(row, col, "%*s", 15, buff);
        col += 15;
        scrn_printf(row, col, "%5d", len);

        if ( skip && (type < ETHER_TYPE_IPv4) )
        	scrn_printf(row, col+7, "<<< Skip %04x", type);
        else if ( skip && (type != ETHER_TYPE_IPv4) )
        	scrn_printf(row, col+7, " EthType %04x", type);
        row++;
    }
leave:
    display_dashline(row+2);

    pktgen.flags &= ~PRINT_LABELS_FLAG;
}

/**************************************************************************//**
*
* pktgen_print_range - Display the range data page.
*
* DESCRIPTION
* Display the range data page on the screen.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_print_range(void)
{
    port_info_t * info;
    range_info_t * range;
    unsigned int pid, col, sp;
    char buff[32];
    unsigned int row;
    struct ether_addr eaddr;
	char	str[64];

    display_topline("** Range Page **");
    scrn_printf(1, 3, "Ports %d-%d of %d", pktgen.starting_port, (pktgen.ending_port - 1), pktgen.nb_ports);

    row = PORT_STATE_ROW;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "Port #");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "dst.ip");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    min");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "src.ip");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    min");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "dst.port / inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "     min / max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "src.port / inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "     min / max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "vlan.id / inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    min / max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "pkt.size / inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "     min / max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "dst.mac");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    min");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    max");

    row++;
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "src.mac");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    inc");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    min");
    scrn_printf(row++, 1, "%-*s", COLUMN_WIDTH_0, "    max");

    // Get the last location to use for the window starting row.
    pktgen.last_row = ++row;
    display_dashline(pktgen.last_row);

    // Display the colon after the row label.
    for(row = 3; row < (pktgen.last_row-1); row++)
        scrn_printf(row, COLUMN_WIDTH_0-1, ":");

    sp = pktgen.starting_port;
    for (pid = 0; pid < pktgen.nb_ports_per_page; pid++) {
        if ( wr_get_map(pktgen.l2p, pid+sp, RTE_MAX_LCORE) == 0 )
            continue;

        info = &pktgen.info[pid+sp];

        // Display Port information Src/Dest IP addr, Netmask, Src/Dst MAC addr
        col = (COLUMN_WIDTH_1 * pid) + COLUMN_WIDTH_0;
        row = PORT_STATE_ROW;

        // Display the port number for the column
        snprintf(buff, sizeof(buff), "Port-%d", pid+sp);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

        range = &info->range;
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->dst_ip), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->dst_ip_inc), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->dst_ip_min), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->dst_ip_max), 0xFFFFFFFF));

        row++;
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->src_ip), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->src_ip_inc), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->src_ip_min), 0xFFFFFFFF));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(range->src_ip_max), 0xFFFFFFFF));

        row++;
		snprintf(str, sizeof(str), "%5d/%5d", range->dst_port, range->dst_port_inc);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);
		snprintf(str, sizeof(str), "%5d/%5d", range->dst_port_min, range->dst_port_max);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);

        row++;
		snprintf(str, sizeof(str), "%5d/%5d", range->src_port, range->src_port_inc);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);
		snprintf(str, sizeof(str), "%5d/%5d", range->src_port_min, range->src_port_max);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);

        row++;
		snprintf(str, sizeof(str), "%4d/%4d", range->vlan_id, range->vlan_id_inc);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);
		snprintf(str, sizeof(str), "%4d/%4d", range->vlan_id_min, range->vlan_id_max);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);

        row++;
		snprintf(str, sizeof(str), "%4d/%4d", range->pkt_size+FCS_SIZE, range->pkt_size_inc);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);
		snprintf(str, sizeof(str), "%4d/%4d", range->pkt_size_min+FCS_SIZE, range->pkt_size_max+FCS_SIZE);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, str);

        row++;
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->dst_mac, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->dst_mac_inc, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->dst_mac_min, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->dst_mac_max, &eaddr)));

        row++;
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->src_mac, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->src_mac_inc, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->src_mac_min, &eaddr)));
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), inet_h64tom(range->src_mac_max, &eaddr)));
    }

    pktgen.flags &= ~PRINT_LABELS_FLAG;
}

/**************************************************************************//**
*
* pktgen_get_link_status - Get the port link status.
*
* DESCRIPTION
* Try to get the link status of a port. The <wait> flag if set tells the
* routine to try and wait for the link status for 3 seconds. If the <wait> flag
* is zero the try three times to get a link status if the link is not up.
*
* RETURNS: N/A
*
* SEE ALSO:
*/
static __inline__ void
pktgen_get_link_status(port_info_t * info, int pid, int wait) {

	int		i;

    /* get link status */
    for(i = 0; i < 3; i++) {
		memset(&info->link, 0, sizeof(info->link));
		rte_eth_link_get_nowait(pid, &info->link);
		if ( info->link.link_status )
			return;
		if ( wait )
			rte_delay_ms(1000);
	}
	// Setup a few default values to prevent problems later.
	info->link.link_speed	= 100;
	info->link.link_duplex	= ETH_LINK_FULL_DUPLEX;
}

/**************************************************************************//**
*
* pktgen_page_stats - Display the statistics on the screen for all ports.
*
* DESCRIPTION
* Display the port statistics on the screen for all ports.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_stats(void)
{
    port_info_t * info;
    unsigned int pid, col, row;
    unsigned sp;
    char buff[32];

    if ( pktgen.flags & PRINT_LABELS_FLAG )
        pktgen_print_static_data();

    memset(&pktgen.cumm_rate_totals, 0, sizeof(eth_stats_t));

    sp = pktgen.starting_port;
    for (pid = 0; pid < pktgen.nb_ports_per_page; pid++) {
        if ( wr_get_map(pktgen.l2p, pid+sp, RTE_MAX_LCORE) == 0 )
            continue;

        info = &pktgen.info[pid + sp];

        // Display the disable string when port is not enabled.
        col = (COLUMN_WIDTH_1 * pid) + COLUMN_WIDTH_0;
        row = PORT_STATE_ROW;

        // Display the port number for the column
        snprintf(buff, sizeof(buff), "%s:%d", pktgen_flags_string(info), pid+sp);
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, buff);

        row = LINK_STATE_ROW;

        // Grab the link state of the port and display Duplex/Speed and UP/Down
        pktgen_get_link_status(info, pid, 0);

        pktgen_link_state(pid, buff, sizeof(buff));
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, buff);

        // Rx/Tx pkts/s rate
        row = LINK_STATE_ROW + 1;
        scrn_printf(row++,  col, "%*llu", COLUMN_WIDTH_1, info->rate_stats.ipackets);
        scrn_printf(row++,  col, "%*llu", COLUMN_WIDTH_1, info->rate_stats.opackets);

        snprintf(buff, sizeof(buff), "%lu/%lu",
        		iBitsTotal(info->rate_stats)/Million, oBitsTotal(info->rate_stats)/Million);
        scrn_printf(row++,  col, "%*s", COLUMN_WIDTH_1, buff);

        pktgen.cumm_rate_totals.ipackets += info->rate_stats.ipackets;
        pktgen.cumm_rate_totals.opackets += info->rate_stats.opackets;
        pktgen.cumm_rate_totals.ibytes += info->rate_stats.ibytes;
        pktgen.cumm_rate_totals.obytes += info->rate_stats.obytes;
        pktgen.cumm_rate_totals.ierrors += info->rate_stats.ierrors;
        pktgen.cumm_rate_totals.oerrors += info->rate_stats.oerrors;

        // Packets Sizes
        row = PKT_SIZE_ROW;
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes.broadcast);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes.multicast);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._64);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._65_127);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._128_255);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._256_511);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._512_1023);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->sizes._1024_1518);
        scrn_snprintf(buff, sizeof(buff), "%llu/%llu", info->sizes.runt, info->sizes.jumbo);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

        // Rx/Tx Errors
        row = PKT_TOTALS_ROW;
        scrn_snprintf(buff, sizeof(buff), "%llu/%llu", info->port_stats.ierrors, info->port_stats.oerrors);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

        // Total Rx/Tx
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->port_stats.ipackets);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, info->port_stats.opackets);

        // Total Rx/Tx mbits
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, iBitsTotal(info->port_stats)/Million);
        scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, oBitsTotal(info->port_stats)/Million);

        scrn_snprintf(buff, sizeof(buff), "%llu/%llu", info->stats.arp_pkts, info->stats.echo_pkts);
        scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);

		if ( pktgen.flags & TX_DEBUG_FLAG ) {
			scrn_snprintf(buff, sizeof(buff), "%llu", info->stats.tx_failed);
			scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);
			scrn_snprintf(buff, sizeof(buff), "%llu/%llu", info->tx_pps, info->tx_cycles);
			scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff);
		}
    }

    // Display the total pkts/s for all ports
    col = (COLUMN_WIDTH_1 * pktgen.nb_ports_per_page) + COLUMN_WIDTH_0;
    row = LINK_STATE_ROW + 1;
    scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, pktgen.cumm_rate_totals.ipackets); scrn_eol();
    scrn_printf(row++, col, "%*llu", COLUMN_WIDTH_1, pktgen.cumm_rate_totals.opackets); scrn_eol();
    snprintf(buff, sizeof(buff), "%lu/%lu",
    	    iBitsTotal(pktgen.cumm_rate_totals)/Million, oBitsTotal(pktgen.cumm_rate_totals)/Million);
    scrn_printf(row++, col, "%*s", COLUMN_WIDTH_1, buff); scrn_eol();
}

/**************************************************************************//**
*
* pktgen_page_config - Show the configuration page for pktgen.
*
* DESCRIPTION
* Display the pktgen configuration page. (Not used)
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_config(void)
{
	display_topline("** Configure Page **");

    scrn_center(20, "Need to add the configuration stuff here");
    display_dashline(22);
}

static int
save_uname(char * line, __attribute__ ((unused))int i) {
	pktgen.uname = wr_strdupf(pktgen.uname, line);
	return 0;
}

static void
pktgen_get_uname(void)
{
	do_command("uname -a", save_uname);
}

static __inline__ uint8_t
sct( uint8_t s, uint8_t c, uint8_t t) {
	lc_info_t	* lc = &pktgen.core_info[0];
	uint8_t		i;

	for(i=0; i<pktgen.core_cnt; i++, lc++) {
		if ( lc->s.socket_id == s && lc->s.core_id == c && lc->s.thread_id == t )
			return lc->s.id;
	}

	return 0;
}

/**************************************************************************//**
*
* pktgen_page_cpu - Display the CPU data page.
*
* DESCRIPTION
* Display the CPU data page for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_cpu()
{
    uint32_t    i, j, row, col, cnt, nb_sockets, nb_cores, nb_threads;
	static int counter = 0;
	lc_info_t	* lc;
    char buff[64];

    display_topline("** CPU Information Page **");

    row = PORT_STATE_ROW;
    col = 1;

    pktgen_get_uname();
    memset(&pktgen.core_info, 0xff, (sizeof(lc_info_t) * RTE_MAX_LCORE));
    cnt = wr_coremap("array", pktgen.core_info, RTE_MAX_LCORE, NULL);
    pktgen.lscpu		= wr_lscpu_info(NULL, NULL);
    pktgen.core_cnt		= cnt;

    nb_sockets = wr_coremap_cnt(pktgen.core_info, cnt, 0);
    nb_cores = wr_coremap_cnt(pktgen.core_info, cnt, 1);
    nb_threads = wr_coremap_cnt(pktgen.core_info, cnt, 2);

    if ( (counter++ & 3) != 0 )
    	return;

    row = 3;
    scrn_printf(row++, 1, "Kernel: %s", pktgen.uname);
    row++;
    scrn_printf(row++, 1, "Model Name: %s", pktgen.lscpu->model_name);
    scrn_printf(row++, 1, "CPU Speed : %s", pktgen.lscpu->cpu_mhz);
    scrn_printf(row++, 1, "Cache Size: %s", pktgen.lscpu->cache_size);
    row++;
    scrn_printf(row++, 1, "CPU Flags : %s", pktgen.lscpu->cpu_flags);
    row += 4;

    scrn_printf(row++, 5, "%d sockets, %d cores per socket and %d threads per core.",
    	nb_sockets, nb_cores, nb_threads);
    scrn_printf(row++, 3, "Socket   : ");
    for(i = 0; i< nb_sockets; i++)
    	printf("%4d      ", i);

    lc = &pktgen.core_info[0];
	for(i = 0; i< nb_cores; i++) {
		scrn_printf(row++, 1, "  Core %3d : [%2d,%2d]   ", i, sct(0, i, 0),   sct(0, i, 1));
		if ( nb_sockets > 1 )
			printf("[%2d,%2d]   ", sct(1, i, 0), sct(1, i, 1));
		if ( nb_sockets > 2 )
			printf("[%2d,%2d]   ", sct(2, i, 0), sct(2, i, 1));
		if ( nb_sockets > 3 )
			printf("[%2d,%2d]   ", sct(3, i, 0), sct(3, i, 1));
		printf("\n");
	}
	wr_port_matrix_dump(pktgen.l2p);

	if ( pktgen.flags & PRINT_LABELS_FLAG ) {

		pktgen.last_row = 36;
	    display_dashline(pktgen.last_row);

		scrn_setw(pktgen.last_row);
		scrn_printf(100, 1, "");        // Put cursor on the last row.
	}
    pktgen.flags &= ~PRINT_LABELS_FLAG;
}

/**************************************************************************//**
*
* pktgen_page_pcap - Display the PCAP data page.
*
* DESCRIPTION
* Display the PCAP data page for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_pcap(uint16_t pid)
{
    if ( pktgen.flags & PRINT_LABELS_FLAG )
        pktgen_print_pcap(pid);
}

/**************************************************************************//**
*
* pktgen_page_range - Display the range data page.
*
* DESCRIPTION
* Display the range data page for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_range(void)
{
    if ( pktgen.flags & PRINT_LABELS_FLAG )
        pktgen_print_range();
}

/**************************************************************************//**
*
* pktgen_page_seq - Display the sequence port data on the screen.
*
* DESCRIPTION
* For a given port display the sequence packet data.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_page_seq(uint32_t pid)
{
    uint32_t    i, row, col;
    port_info_t * info;
    pkt_seq_t   * pkt;
    char buff[64];

    display_topline("** Sequence Page **");

    info = &pktgen.info[pid];

    row = PORT_STATE_ROW;
    col = 1;
    scrn_printf(row++, col, "Port: %2d, Sequence Count: %2d of %2d  ", pid, info->seqCnt, NUM_SEQ_PKTS);
    scrn_printf(row++, col, "%*s %*s%*s%*s%*s%*s%*s%*s",
            6, "Seq:",
            COLUMN_WIDTH_0, "Dst MAC",
            COLUMN_WIDTH_0, "Src MAC",
            COLUMN_WIDTH_0, "Dst IP",
            COLUMN_WIDTH_0+2, "Src IP",
            12, "Port S/D",
            15, "Protocol:VLAN",
            5, "Size");
    for(i = 0; i < NUM_SEQ_PKTS; i++) {
        col = 1;
        pkt = &info->seq_pkt[i];

        if ( i >= info->seqCnt ) {
        	scrn_eol_pos(row++, col);
        	continue;
        }

        scrn_printf(row, col, "%5d:", i);
        col += 7;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &pkt->eth_dst_addr));
        col += COLUMN_WIDTH_1;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_mtoa(buff, sizeof(buff), &pkt->eth_src_addr));
        col += COLUMN_WIDTH_1;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1, inet_ntop4(buff, sizeof(buff), htonl(pkt->ip_dst_addr), 0xFFFFFFFF));
        col += COLUMN_WIDTH_1;
        scrn_printf(row, col, "%*s", COLUMN_WIDTH_1+2, inet_ntop4(buff, sizeof(buff), htonl(pkt->ip_src_addr), pkt->ip_mask));
        col += COLUMN_WIDTH_1+2;

        snprintf(buff, sizeof(buff), "%d/%d", pkt->sport, pkt->dport);
        scrn_printf(row, col, "%*s", 12, buff);
        col += 12;
        snprintf(buff, sizeof(buff), "%s/%s:%04x", (pkt->ethType == ETHER_TYPE_IPv4)? "IPv4" :
                                                      (pkt->ethType == ETHER_TYPE_IPv6)? "IPv6" : "Other",
                                                      (pkt->ipProto == PG_IPPROTO_TCP)? "TCP" :
                                                      (pkt->ipProto == PG_IPPROTO_ICMP)? "ICMP" : "UDP",
                                                    		  pkt->vlanid);
        scrn_printf(row, col, "%*s", 15, buff);
        col += 15;
        scrn_printf(row, col, "%5d", pkt->pktSize+FCS_SIZE);
        row++;
    }

    display_dashline(row+2);
}

/**************************************************************************//**
*
* pktgen_print_packet_dump - Print captured packets to the screen
*
* DESCRIPTION
* When some packets are captured on user request, print the packet data to
* the screen.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_print_packet_dump(void)
{
	port_info_t * info;
	struct cmdline *cl = pktgen.cl;

	unsigned int pid;
	unsigned int i, j;
	unsigned char * pdata;
	uint32_t plen;

	for (pid = 0; pid < RTE_MAX_ETHPORTS; pid++) {
		if ( wr_get_map(pktgen.l2p, pid, RTE_MAX_LCORE) == 0 )
			continue;

		info = &pktgen.info[pid];
		for (; info->dump_head < info->dump_tail; ++info->dump_head) {
			pdata = (unsigned char *)info->dump_list[info->dump_head].data;
			plen = info->dump_list[info->dump_head].len;

			fprintf(stderr, "Port %d, packet with length %d:\n", pid, plen);

			for (i = 0; i < plen; i += 16) {
				/* Byte counter */
				fprintf(stderr, "%06x: ", i);

				for (j = 0; j < 16; ++j) {
					/* Hex. value of character */
					if (i + j < plen)
					    fprintf(stderr, "%02x ", pdata[i + j]);
					else
					    fprintf(stderr, "   ");

					/* Extra padding after 8 hex values for readability */
					if ((j + 1) % 8 == 0)
					    fprintf(stderr, " ");
				}

				/* Separate hex. values and raw characters */
				fprintf(stderr, "\t");

				for (j = 0; j < 16; ++j) {
					if (i + j < plen)
						fprintf(stderr, "%c", isprint(pdata[i + j]) ? pdata[i + j] : '.');
				}

				fprintf(stderr, "\n");
			}

			rte_free(info->dump_list[info->dump_head].data);
			info->dump_list[info->dump_head].data = NULL;
		}
	}
}

/**************************************************************************//**
*
* pktgen_page_display - Display the correct page based on timer0 callback.
*
* DESCRIPTION
* When timer0 is active update or display the correct page of data.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_page_display(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
    static unsigned int counter = 0;

    // Leave if the screen is paused
    if ( scrn_is_paused() )
        return;

    scrn_save();

    scrn_printf(1,1, "%c", "-\\|/"[(counter++ & 3)]);

    if ( pktgen.flags & CPU_PAGE_FLAG )
        pktgen_page_cpu();
    else if ( pktgen.flags & PCAP_PAGE_FLAG )
        pktgen_page_pcap(pktgen.portNum);
    else if ( pktgen.flags & RANGE_PAGE_FLAG )
        pktgen_page_range();
    else if ( pktgen.flags & CONFIG_PAGE_FLAG )
        pktgen_page_config();
    else if ( pktgen.flags & SEQUENCE_PAGE_FLAG )
        pktgen_page_seq(pktgen.portNum);
    else
        pktgen_page_stats();

    scrn_restore();

    pktgen_print_packet_dump();
}

/**************************************************************************//**
*
* pktgen_process_stats - Process statistics for all ports on timer1
*
* DESCRIPTION
* When timer1 callback happens then process all of the port statistics.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_process_stats(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
    unsigned int pid;
    struct rte_eth_stats    stats;
    port_info_t * info;
    static unsigned int counter = 0;

    counter++;
    if ( pktgen.flags & BLINK_PORTS_FLAG ) {
    	for (pid = 0; pid < pktgen.nb_ports; pid++) {
    		if ( (pktgen.blinklist & (1ULL << pid)) == 0 )
    			continue;

			if ( counter & 1 )
				rte_eth_led_on(pid);
			else
				rte_eth_led_off(pid);
    	}
    }

    for (pid = 0; pid < pktgen.nb_ports; pid++) {
        if ( wr_get_map(pktgen.l2p, pid, RTE_MAX_LCORE) == 0 )
            continue;

        info = &pktgen.info[pid];

        rte_eth_stats_get(pid, &stats);

        // Normalize counts to the initial state, used for clearing statistics
        stats.ipackets  -= info->init_stats.ipackets;
        stats.opackets  -= info->init_stats.opackets;
        stats.ibytes    -= info->init_stats.ibytes;
        stats.obytes    -= info->init_stats.obytes;
        stats.ierrors   -= info->init_stats.ierrors;
        stats.oerrors   -= info->init_stats.oerrors;

        info->rate_stats.ipackets   = stats.ipackets - info->port_stats.ipackets;
        info->rate_stats.opackets   = stats.opackets - info->port_stats.opackets;
        info->rate_stats.ibytes     = stats.ibytes - info->port_stats.ibytes;
        info->rate_stats.obytes     = stats.obytes - info->port_stats.obytes;
        info->rate_stats.ierrors    = stats.ierrors - info->port_stats.ierrors;
        info->rate_stats.oerrors    = stats.oerrors - info->port_stats.oerrors;

        // Use structure move to copy the data.
        *(struct rte_eth_stats *)&info->port_stats = *(struct rte_eth_stats *)&stats;
    }
}

static struct rte_timer timer0;
static struct rte_timer timer1;

/**************************************************************************//**
*
* pktgen_timer_setup - Set up the timer callback routines.
*
* DESCRIPTION
* Setup the two timers to be used for display and calculating statistics.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
rte_timer_setup(void)
{
    int lcore_id = rte_get_master_lcore();

    /* init RTE timer library */
    rte_timer_subsystem_init();

    /* init timer structures */
    rte_timer_init(&timer0);
    rte_timer_init(&timer1);

    /* load timer0, every 2 seconds, on Display lcore, reloaded automatically */
    rte_timer_reset(&timer0, (pktgen.hz*2), PERIODICAL, lcore_id, pktgen_page_display, NULL);

    /* load timer1, every second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer1, pktgen.hz, PERIODICAL, lcore_id, pktgen_process_stats, NULL);
}

/**************************************************************************//**
*
* pktgen_mbuf_pool_create - Create mbuf packet pool.
*
* DESCRIPTION
* Callback routine for creating mbuf packets from a mempool.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static struct rte_mempool *
pktgen_mbuf_pool_create(const char * type, uint8_t pid, uint8_t queue_id,
		uint32_t nb_mbufs, int socket_id, int cache_size )
{
    struct rte_mempool * mp;
    uint32_t	size = MBUF_SIZE;
    char    name[RTE_MEMZONE_NAMESIZE];

    snprintf(name, sizeof(name), "%-12s%u:%u", type, pid, queue_id);
    printf_info("    Create: %-*s - Memory used (MBUFs %4u x (size %u + Hdr %lu)) + %lu = %6lu KB\n",
            16, name, nb_mbufs, size, sizeof(struct rte_mbuf), sizeof(struct rte_mempool),
            (((nb_mbufs * (size + sizeof(struct rte_mbuf)) + sizeof(struct rte_mempool))) + 1023)/1024);
    pktgen.mem_used += ((nb_mbufs * (size + sizeof(struct rte_mbuf)) + sizeof(struct rte_mempool)));
    pktgen.total_mem_used += ((nb_mbufs * (size + sizeof(struct rte_mbuf)) + sizeof(struct rte_mempool)));

    /* create the mbuf pool */
    mp = rte_mempool_create(name, nb_mbufs, size, cache_size,
                   sizeof(struct rte_pktmbuf_pool_private),
                   rte_pktmbuf_pool_init, (void *)((uint64_t)size),
                   rte_pktmbuf_init, NULL,
                   socket_id, MEMPOOL_F_DMA);
    if (mp == NULL)
        rte_panic("Cannot create mbuf pool (%s) port %d, queue %d, nb_mbufs %d, socket_id %d\n%s\n",
        		name, pid, queue_id, nb_mbufs, socket_id, rte_strerror(errno));

    return mp;
}

/**************************************************************************//**
*
* pktgen_range_setup - Setup the default values for a range port.
*
* DESCRIPTION
* Setup the default range data for a given port.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_range_setup(port_info_t * info)
{
	range_info_t * range = &info->range;

	range->dst_ip       = IPv4(192, 168, info->pid+1, 1);
	range->dst_ip_min   = IPv4(192, 168, info->pid+1, 1);
	range->dst_ip_max   = IPv4(192, 168, info->pid+1, 254);
	range->dst_ip_inc	= 0x00000001;

	range->src_ip		= IPv4(192, 168, info->pid, 1);
	range->src_ip_min	= IPv4(192, 168, info->pid, 1);
	range->src_ip_max	= IPv4(192, 168, info->pid, 254);
	range->src_ip_inc	= 0x00000000;

	range->dst_port		= (info->pid << 8);
	range->dst_port_inc	= 0x0001;
	range->dst_port_min	= range->dst_port + 0;
	range->dst_port_max	= range->dst_port + 254;

	range->src_port		= (info->pid << 8);
	range->src_port_inc	= 0x0001;
	range->src_port_min	= range->src_port + 0;
	range->src_port_max	= range->src_port + 254;

	range->vlan_id		= info->vlanid;
	range->vlan_id_inc	= 0;
	range->vlan_id_min	= MIN_VLAN_ID;
	range->vlan_id_max	= MAX_VLAN_ID;

	range->pkt_size		= MIN_PKT_SIZE;
	range->pkt_size_inc	= 0;
	range->pkt_size_min	= MIN_PKT_SIZE;
	range->pkt_size_max	= MAX_PKT_SIZE;

	info->seq_pkt[RANGE_PKT].pktSize = MIN_PKT_SIZE;

	inet_mtoh64(&info->seq_pkt[SINGLE_PKT].eth_dst_addr, &range->dst_mac);
    memset(&range->dst_mac_inc, 0, sizeof (range->dst_mac_inc));
    memset(&range->dst_mac_min, 0, sizeof (range->dst_mac_min));
    memset(&range->dst_mac_max, 0, sizeof (range->dst_mac_max));

	inet_mtoh64(&info->seq_pkt[SINGLE_PKT].eth_src_addr, &range->src_mac);
    memset(&range->src_mac_inc, 0, sizeof (range->src_mac_inc));
    memset(&range->src_mac_min, 0, sizeof (range->src_mac_min));
    memset(&range->src_mac_max, 0, sizeof (range->src_mac_max));
}

/**************************************************************************//**
*
* pktgen_config_ports - Configure the ports for RX and TX
*
* DESCRIPTION
* Handle setting up the ports in DPDK.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void pktgen_config_ports(void)
{
    uint32_t lid, pid, i, s, q, sid;
	uint64_t	k;
    rxtx_t	rt;
    pkt_seq_t   * pkt;
    port_info_t     * info;
    char buff[RTE_MEMZONE_NAMESIZE];
    int32_t ret, cache_size;
    struct rte_eth_txconf tx;
	char memzone_name[RTE_MEMZONE_NAMESIZE];

    // Get a local copy the tx configure information.
    memcpy(&tx, &tx_conf, sizeof(struct rte_eth_txconf));

    pktgen.coremask = wr_get_coremask(&k);

    // Find out the total number of ports in the system.
    // We have already blacklisted the ones we needed to in main routine.
    pktgen.nb_ports = rte_eth_dev_count();
    if (pktgen.nb_ports > RTE_MAX_ETHPORTS)
        pktgen.nb_ports = RTE_MAX_ETHPORTS;

    if ( pktgen.nb_ports == 0 )
    	rte_panic("*** Did not find any ports to use ***\n");

    pktgen.starting_port = 0;

    // Setup the number of ports to display at a time
	if ( pktgen.nb_ports > pktgen.nb_ports_per_page )
		pktgen.ending_port = pktgen.starting_port + pktgen.nb_ports_per_page;
	else
		pktgen.ending_port = pktgen.starting_port + pktgen.nb_ports;

    wr_port_matrix_dump(pktgen.l2p);

    printf_info("Configuring %d ports, MBUF Size %d, MBUF Cache Size %d\n",
    		pktgen.nb_ports, MBUF_SIZE, MBUF_CACHE_SIZE);

    // For each lcore setup each port that is handled by that lcore.
    for(lid = 0; lid < RTE_MAX_LCORE; lid++) {

        if ( wr_get_map(pktgen.l2p, RTE_MAX_ETHPORTS, lid) == 0 )
            continue;

		// For each port attached or handled by the lcore
        for(pid = 0; pid < pktgen.nb_ports; pid++) {

        	// If non-zero then this port is handled by this lcore.
            if ( wr_get_map(pktgen.l2p, pid, lid) == 0 )
                continue;
        	wr_set_port_private(pktgen.l2p, pid, &pktgen.info[pid]);
        	pktgen.info[pid].pid = pid;
        }
    }
    wr_dump_l2p(pktgen.l2p);

    pktgen.total_mem_used = 0;

    for(k = 0, pid = 0; pid < pktgen.nb_ports; pid++) {
    	// Skip if we do not have any lcores attached to a port.
    	if ( (rt.rxtx = wr_get_map(pktgen.l2p, pid, RTE_MAX_LCORE)) == 0 )
            continue;

		printf_info("Initialize Port %d -- TxQ %d, RxQ %d,  ", pid, rt.tx, rt.rx);

        info = wr_get_port_private(pktgen.l2p, pid);

		// Create the pkt header structures for transmitting sequence of packets.
		snprintf(buff, sizeof(buff), "seq_hdr_%d", pid);
		info->seq_pkt = (pkt_seq_t *)rte_zmalloc(buff, (sizeof(pkt_seq_t) * NUM_TOTAL_PKTS), CACHE_LINE_SIZE);
		if ( info->seq_pkt == NULL )
			rte_panic("Unable to allocate %d pkt_seq_t headers", NUM_TOTAL_PKTS);

		info->seqIdx    = 0;
		info->seqCnt    = 0;

		info->nb_mbufs  = MAX_MBUFS_PER_PORT;
		cache_size = (info->nb_mbufs > RTE_MEMPOOL_CACHE_MAX_SIZE)?
							RTE_MEMPOOL_CACHE_MAX_SIZE : info->nb_mbufs;

		if ( (ret = rte_eth_dev_configure(pid, rt.rx, rt.tx, &port_conf)) < 0)
			rte_panic("Cannot configure device: port=%d, Num queues %d,%d (%d)%s\n",
					pid, rt.rx, rt.tx, errno, rte_strerror(-ret));

		pkt = &info->seq_pkt[SINGLE_PKT];

		// Grab the source MAC addresses */
		rte_eth_macaddr_get(pid, &pkt->eth_src_addr);
		printf_info("Src MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				pkt->eth_src_addr.addr_bytes[0],
				pkt->eth_src_addr.addr_bytes[1],
				pkt->eth_src_addr.addr_bytes[2],
				pkt->eth_src_addr.addr_bytes[3],
				pkt->eth_src_addr.addr_bytes[4],
				pkt->eth_src_addr.addr_bytes[5]);

		// Copy the first Src MAC address in SINGLE_PKT to the rest of the sequence packets.
		for (i = 0; i < NUM_SEQ_PKTS; i++)
			ethAddrCopy( &info->seq_pkt[i].eth_src_addr, &pkt->eth_src_addr );

		pktgen.mem_used = 0;

		for(q = 0; q < rt.rx; q++) {
			// grab the socket id value based on the lcore being used.
			sid		= rte_lcore_to_socket_id(wr_get_port_lid(pktgen.l2p, pid, q));

			// Create and initialize the default Receive buffers.
			info->q[q].rx_mp = pktgen_mbuf_pool_create("Default RX", pid, q, info->nb_mbufs, sid, cache_size);
			if ( info->q[q].rx_mp == NULL )
				rte_panic("Cannot init port %d for Default RX mbufs\n", pid);

			ret = rte_eth_rx_queue_setup(pid, q, pktgen.nb_rxd, sid, &rx_conf, pktgen.info[pid].q[q].rx_mp);
			if (ret < 0)
				rte_panic("rte_eth_rx_queue_setup: err=%d, port=%d, %s\n", ret, pid, rte_strerror(-ret));
		}
		printf_info("\n");

		for(q = 0; q < rt.tx; q++) {
			// grab the socket id value based on the lcore being used.
			sid		= rte_lcore_to_socket_id(wr_get_port_lid(pktgen.l2p, pid, q));

			// Create and initialize the default Transmit buffers.
			info->q[q].tx_mp = pktgen_mbuf_pool_create("Default TX", pid, q, MAX_MBUFS_PER_PORT, sid, cache_size);
			if ( info->q[q].tx_mp == NULL )
				rte_panic("Cannot init port %d for Default TX mbufs\n", pid);

			// Create and initialize the range Transmit buffers.
			info->q[q].range_mp = pktgen_mbuf_pool_create("Range TX", pid, q, MAX_MBUFS_PER_PORT,	sid, 0);
			if ( info->q[q].range_mp == NULL )
				rte_panic("Cannot init port %d for Range TX mbufs\n", pid);

			// Create and initialize the sequence Transmit buffers.
			info->q[q].seq_mp = pktgen_mbuf_pool_create("Sequence TX", pid, q, MAX_MBUFS_PER_PORT, sid, cache_size);
			if ( info->q[q].seq_mp == NULL )
				rte_panic("Cannot init port %d for Sequence TX mbufs\n", pid);

			// Used for sending special packets like ARP requests
			info->q[q].special_mp = pktgen_mbuf_pool_create("Special TX", pid, q, MAX_SPECIAL_MBUFS, sid, cache_size);
			if (info->q[q].special_mp == NULL)
				rte_panic("Cannot init port %d for Special TX mbufs\n", pid);

			// Setup the PCAP file for each port
			if ( pktgen.info[pid].pcap != NULL ) {
				if ( pktgen_pcap_parse(pktgen.info[pid].pcap, info, q) == -1 )
					rte_panic("Cannot load PCAP file for port %d", pid);
			}
			// Find out the link speed to program the WTHRESH value correctly.
			pktgen_get_link_status(info, pid, 0);

			tx.tx_thresh.wthresh = (info->link.link_speed == 1000)? TX_WTHRESH_1GB : TX_WTHRESH;

			ret = rte_eth_tx_queue_setup(pid, q, pktgen.nb_txd, sid, &tx);
			if (ret < 0)
				rte_panic("rte_eth_tx_queue_setup: err=%d, port=%d, %s\n", ret, pid, rte_strerror(-ret));
#if 0
			ret = rte_eth_dev_flow_ctrl_set(pid, &fc_conf);
			if (ret < 0)
				rte_panic("rte_eth_dev_flow_ctrl_set: err=%d, port=%d, %s\n", ret, pid, rte_strerror(-ret));
#endif
			printf_info("\n");
		}
		printf_info("%*sPort memory used = %6lu KB\n", 71, " ", (pktgen.mem_used + 1023)/1024);
	}
    printf_info("%*sTotal memory used = %6lu KB\n", 70, " ", (pktgen.total_mem_used + 1023)/1024);

    // Start up the ports and display the port Link status
    for(k = 0, pid = 0; pid < pktgen.nb_ports; pid++) {
        if ( wr_get_map(pktgen.l2p, pid, RTE_MAX_LCORE) == 0 )
            continue;

        info = wr_get_port_private(pktgen.l2p, pid);

        /* Start device */
        if ( (ret = rte_eth_dev_start(pid)) < 0 )
            rte_panic("rte_eth_dev_start: port=%d, %s\n", pid, rte_strerror(-ret));

        pktgen_get_link_status(info, pid, 1);

        if (info->link.link_status) {
            printf_info("Port %2d: Link Up - speed %u Mbps - %s", pid,
                   (uint32_t) info->link.link_speed,
                   (info->link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
                   ("full-duplex") : ("half-duplex"));
        } else
            printf_info("Port %2d: Link Down", pid);


        // If enabled, put device in promiscuous mode.
        if (pktgen.flags & PROMISCUOUS_ON_FLAG) {
        	printf_info(" <Enable promiscuous mode>");
            rte_eth_promiscuous_enable(pid);
        }
        printf_info("\n");

    	pktgen.info[pid].seq_pkt[SINGLE_PKT].pktSize = MIN_PKT_SIZE;

        // Setup the port and packet defaults. (must be after link speed is found)
        for (s = 0; s < NUM_TOTAL_PKTS; s++)
            pktgen_port_defaults(pid, s);

        pktgen_range_setup(info);
    }

	for (sid = 0; sid < RTE_MAX_NUMA_NODES; sid++) {
		pktgen.capture[sid].lcore = RTE_MAX_LCORE;
		pktgen.capture[sid].port = RTE_MAX_ETHPORTS;
		pktgen.capture[sid].mem_used = 0;

		// TODO: use rte_snprintf() ?
		snprintf(memzone_name, sizeof(memzone_name), "Capture_MZ_%d", sid);
		pktgen.capture[sid].mz = rte_memzone_reserve(memzone_name, 0, sid,
				RTE_MEMZONE_1GB | RTE_MEMZONE_SIZE_HINT_ONLY);

		if (pktgen.capture[sid].mz == NULL)
			continue;
	}
}

/**************************************************************************//**
*
* pktgen_pcap_mbuf_ctor - Callback routine to construct PCAP packets.
*
* DESCRIPTION
* Callback routine to construct a set of PCAP packet buffers.
*
* RETURNS: N/A
*
* SEE ALSO:
*/
static void
pktgen_pcap_mbuf_ctor(struct rte_mempool *mp, void *opaque_arg, void *_m, unsigned i)
{
    struct rte_mbuf *m = _m;
	uint32_t	buf_len = mp->elt_size - sizeof(struct rte_mbuf);
	uint16_t	type;
    pcaprec_hdr_t hdr;
    ssize_t	len = -1;
    char buffer[2048];
    pcap_info_t * pcap = (pcap_info_t *)opaque_arg;

	RTE_MBUF_ASSERT(mp->elt_size >= sizeof(struct rte_mbuf));

	memset(m, 0, mp->elt_size);

    /* start of buffer is just after mbuf structure */
    m->buf_addr		= (char *)m + sizeof(struct rte_mbuf);
    m->buf_physaddr = rte_mempool_virt2phy(mp, m->buf_addr);
    m->buf_len		= (uint16_t)buf_len;

    /* keep some headroom between start of buffer and data */
    m->pkt.data = (char *)m->buf_addr + RTE_MIN(RTE_PKTMBUF_HEADROOM, m->buf_len);

    /* init some constant fields */
    m->type         = RTE_MBUF_PKT;
    m->pool         = mp;
    m->pkt.nb_segs  = 1;
    m->pkt.in_port	= 0xff;
    m->ol_flags		= 0;

    for(;;) {
        if ( (i & 0x3ff) == 0 ) {
        	printf_info("%c\b", "-\\|/"[(i >> 10) & 3]);
        	i++;
        }

    	if ( unlikely(wr_pcap_read(pcap, &hdr, buffer, sizeof(buffer)) <= 0) ) {
    		wr_pcap_rewind(pcap);
    		continue;
    	}

        len = hdr.incl_len;

        // Adjust the packet length if not a valid size.
        if ( len < (ETHER_MIN_LEN - 4) )
    		len = (ETHER_MIN_LEN - 4);
        else if ( len > (ETHER_MAX_LEN - 4) )
    		len = (ETHER_MAX_LEN - 4);

        m->pkt.data_len = len;
        m->pkt.pkt_len	= len;

        rte_memcpy((uint8_t *)m->pkt.data, buffer, len);
        break;
    }
}

/**************************************************************************//**
*
* pktgen_pcap_parse - Parse a PCAP file.
*
* DESCRIPTION
* Parse a pcap file into packet buffers.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

int
pktgen_pcap_parse(pcap_info_t * pcap, port_info_t * info, unsigned qid)
{
    pcaprec_hdr_t hdr;
    uint32_t    elt_count, data_size, elt_size, len, i;
    uint64_t	pkt_sizes = 0, rate;
	uint16_t	type;
    char		buffer[2048];
	char		name[RTE_MEMZONE_NAMESIZE];

    if ( (pcap == NULL) || (info == NULL) )
        return -1;

    wr_pcap_rewind(pcap);

	snprintf(name, sizeof(name), "%-12s%d:%d", "PCAP TX", info->pid, 0);
    printf_info("    Process: %-*s ", 18, name);

    pkt_sizes = elt_count = i = 0;

    // The wr_pcap_open left the file pointer to the first packet.
    while( wr_pcap_read(pcap, &hdr, buffer, sizeof(buffer)) > 0 ) {

    	// Skip any jumbo packets or packets that are too small
        len = hdr.incl_len;

        if ( len < (ETHER_MIN_LEN - 4) )
    		len = (ETHER_MIN_LEN - 4);
        else if ( len > (ETHER_MAX_LEN - 4) )
    		len = (ETHER_MAX_LEN - 4);

        elt_count++;

        if ( (elt_count & 0x3ff) == 0 )
        	printf_info("%c\b", "-\\|/"[i++ & 3]);

        pkt_sizes += len;
    }

    // If count is greater then zero then we allocate and create the PCAP mbuf pool.
    if ( elt_count > 0 ) {
    	// Create the average size packet
		info->pcap->pkt_size	= (pkt_sizes / elt_count);
        info->pcap->pkt_count	= elt_count;
        info->pcap->pkt_idx		= 0;

        wr_pcap_rewind(pcap);

        // Round up the count and size to allow for TX ring size.
        if ( elt_count < MAX_MBUFS_PER_PORT )
        	elt_count = MAX_MBUFS_PER_PORT;
        elt_count = rte_align32pow2(elt_count);

        printf_info("\r    Create: %-*s   \b", 16, name);
    	info->q[qid].pcap_mp = rte_mempool_create(name, elt_count, MBUF_SIZE, 0,
                   sizeof(struct rte_pktmbuf_pool_private),
                   rte_pktmbuf_pool_init, (void *)((uint64_t)MBUF_SIZE),
                   pktgen_pcap_mbuf_ctor, (void *)pcap,
                   rte_lcore_to_socket_id(0), MEMPOOL_F_DMA);
        printf_info("\r");
        if ( info->q[qid].pcap_mp == NULL )
            rte_panic("***** Cannot init port %d for PCAP packets\n", info->pid);

        data_size = (info->pcap->pkt_count * MBUF_SIZE);
        printf_info("    Create: %-*s - Number of MBUFs %6u for %5d packets                 = %6u KB\n",
                16, name, elt_count, info->pcap->pkt_count, (data_size + 1023)/1024);
        pktgen.mem_used			+= data_size;
        pktgen.total_mem_used	+= data_size;

        pktgen_set_port_flags(info, SEND_PCAP_PKTS);
    }

    pktgen_packet_rate(info);
    return 0;
}

/**************************************************************************//**
*
* pktgen_l2p_dump - Dump the l2p table
*
* DESCRIPTION
* Dump the l2p table
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_l2p_dump(void)
{
	wr_raw_dump_l2p(pktgen.l2p);
}
/**************************************************************************//**
*
* pktgen_usage - Display the help for the command line.
*
* DESCRIPTION
* Display the help message for the command line.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
pktgen_usage(const char *prgname)
{
    printf_info("Usage: %s [EAL options] -- -p PORTMASK [-h] [-P] [-G] [-f cmd_file] [-s P:PCAP_file] [-m <string>]\n"
           "  -p PORTMASK  hexadecimal bitmask of ports to configure\n"
           "  -s P:file    PCAP packet stream file, 'P' is the port number\n"
           "  -f filename  Command file (.pkt) to execute or a Lua script (.lua) file\n"
           "  -P           Enable PROMISCUOUS mode on all ports\n"
    	   "  -g address   Optional IP address and port number default is (localhost:0x5606)\n"
    	   "               If -g is used that enable socket support as a server application\n"
    	   "  -G           Enable socket support using default server values localhost:0x5606 \n"
    	   "  -N           Enable NUMA support\n"
           "  -m <string>  matrix for mapping ports to logical cores\n"
    	   "      BNF: (or kind of BNF)\n"
    	   "      <matrix-string>   := \"\"\" <lcore-port> { \",\" <lcore-port>} \"\"\"\n"
    	   "      <lcore-port>      := <lcore-list> \".\" <port-list>\n"
    	   "      <lcore-list>      := \"[\" <rx-list> \":\" <tx-list> \"]\"\n"
    	   "      <port-list>       := \"[\" <rx-list> \":\" <tx-list>\"]\"\n"
    	   "      <rx-list>         := <num> { \"/\" (<num> | <list>) }\n"
    	   "      <tx-list>         := <num> { \"/\" (<num> | <list>) }\n"
    	   "      <list>            := <num> { \"/\" (<range> | <list>) }\n"
    	   "      <range>           := <num> \"-\" <num> { \"/\" <range> }\n"
    	   "      <num>             := <digit>+\n"
    	   "      <digit>           := 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9\n"
    	   "      1.0, 2.1, 3.2                 - core 1 handles port 0 rx/tx,\n"
    	   "                                      core 2 handles port 1 rx/tx\n"
    	   "      1.[0-2], 2.3, ...             - core 1 handle ports 0,1,2 rx/tx,\n"
    	   "                                      core 2 handle port 3 rx/tx\n"
    	   "      [0-1].0, [2/4-5].1, ...       - cores 0-1 handle port 0 rx/tx,\n"
    	   "                                      cores 2,4,5 handle port 1 rx/tx\n"
    	   "      [1:2].0, [4:6].1, ...         - core 1 handles port 0 rx,\n"
    	   "                                      core 2 handles port 0 tx,\n"
    	   "      [1:2].[0-1], [4:6].[2/3], ... - core 1 handles port 0 & 1 rx,\n"
    	   "                                      core 2 handles port  0 & 1 tx\n"
    	   "      [1:2-3].0, [4:5-6].1, ...     - core 1 handles port 1 rx, cores 2,3 handle port 0 tx\n"
    	   "                                      core 4 handles port 1 rx & core 5,6 handles port 1 tx\n"
    	   "      [1-2:3].0, [4-5:6].1, ...     - core 1,2 handles port 0 rx, core 3 handles port 0 tx\n"
    	   "                                      core 4,5 handles port 1 rx & core 6 handles port 1 tx\n"
    	   "      [1-2:3-5].0, [4-5:6/8].1, ... - core 1,2 handles port 0 rx, core 3,4,5 handles port 0 tx\n"
    	   "                                      core 4,5 handles port 1 rx & core 6,8 handles port 1 tx\n"
    	   "      [1:2].[0:0-7], [3:4].[1:0-7], - core 1 handles port 0 rx, core 2 handles ports 0-7 tx\n"
    	   "                                      core 3 handles port 1 rx & core 4 handles port 0-7 tx\n"
    	   "      BTW: you can use \"{}\" instead of \"[]\" as it does not matter to the syntax.\n"
           "  -h           Display the help information\n",
           prgname);
}

/**************************************************************************//**
*
* pktgen_parse_args - Main parsing routine for the command line.
*
* DESCRIPTION
* Main parsing routine for the command line.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static int
pktgen_parse_args(int argc, char **argv)
{
    int opt, ret, port;
    char **argvopt;
    int option_index;
    char *prgname = argv[0], * p;
    static struct option lgopts[] = {
        {NULL, 0, 0, 0}
    };

    argvopt = argv;

	pktgen.hostname		= (char *)strdupf(pktgen.hostname, "localhost");
	pktgen.socket_port	= 0x5606;

    pktgen.argc = argc;
    for (opt = 0; opt < argc; opt++)
    	pktgen.argv[opt] = strdup(argv[opt]);

    while ((opt = getopt_long(argc, argvopt, "p:m:f:s:g:hPNG",
                  lgopts, &option_index)) != EOF) {
        switch (opt) {
        case 'p':			// Port mask (required).
            pktgen.enabled_port_mask = wr_parse_portmask(optarg);
            if (pktgen.enabled_port_mask < 0) {
                printf_info("invalid portmask\n");
                pktgen_usage(prgname);
                return -1;
            }
            break;

        case 'f':			// Command file or Lua script.
            pktgen.cmd_filename = strdup(optarg);
            break;

        case 'm':			// Matrix for port mapping.
            if ( wr_parse_matrix(pktgen.l2p, optarg) == -1 ) {
                printf_info("invalid matrix string (%s)\n", optarg);
                pktgen_usage(prgname);
                return -1;
            }
            break;

        case 's':           // Read a PCAP packet capture file (stream)
        	port = strtol(optarg, NULL, 10);
        	p = strchr(optarg, ':');
            if( (p == NULL) || (pktgen.info[port].pcap = wr_pcap_open(++p, port)) == NULL ) {
                printf_info("Invalid PCAP filename (%s) must include port number as P:filename\n", optarg);
                pktgen_usage(prgname);
                return -1;
            }
            break;

        case 'P':			// Enable promiscuous mode on the ports
            pktgen.flags    |= PROMISCUOUS_ON_FLAG;
            break;

        case 'N':			// Enable NUMA support.
        	pktgen.flags	|= NUMA_SUPPORT_FLAG;
        	break;

        case 'G':
        	pktgen.flags	|= (ENABLE_GUI_FLAG | IS_SERVER_FLAG);
        	break;

        case 'g':			// Define the port number and IP address used for the socket connection.
        	pktgen.flags	|= (ENABLE_GUI_FLAG | IS_SERVER_FLAG);

        	p = strchr(optarg, ':');
        	if ( p == NULL )		// No : symbol means pktgen is a server application.
				pktgen.hostname	= (char *)strdupf(pktgen.hostname, optarg);
        	else {
        		char	c = *p;

        		*p = '\0';
    			if ( p != optarg )
					pktgen.hostname = (char *)strdupf(pktgen.hostname, optarg);

	    		pktgen.socket_port = strtol(++p, NULL, 0);
	    		printf_info(">>> Socket GUI support %s%c0x%x\n", pktgen.hostname, c, pktgen.socket_port);
        	}
        	break;

        case 'h':			// print out the help message
            pktgen_usage(prgname);
            return -1;

        case 0:				// Long options
        default:
            pktgen_usage(prgname);
            return -1;
        }
    }

    // If the port mask is not set we exit with usage message.
    if (pktgen.enabled_port_mask == 0) {
        printf_info("*** Error must specify the portmask\n");
        pktgen_usage(prgname);
        return -1;
    }

    // Setup the program name
    if (optind >= 0)
        argv[optind-1] = prgname;

    ret = optind-1;
    optind = 0;				/* reset getopt lib */
    return ret;
}

/**************************************************************************//**
*
* pktgen_load_cmds - Load and execute a command file or Lua script file.
*
* DESCRIPTION
* Load and execute a command file or Lua script file.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

int
pktgen_load_cmds( char * filename )
{
    if ( filename == NULL )
        return 0;

    if ( strstr(filename, ".lua") || strstr(filename, ".LUA") ) {
    	if ( pktgen.L == NULL )
    		return -1;

    	// Execute the Lua script file.
    	if ( luaL_dofile(pktgen.L, filename) != 0 ) {
    		fprintf(stderr,"%s\n", lua_tostring(pktgen.L,-1));
    		return -1;
    	}
    } else {
        FILE    * fd;
        char    buff[256];

		fd = fopen((const char *)filename, "r");
		if ( fd == NULL )
			return -1;

		// Reset the command line system for the script.
		rdline_reset(&pktgen.cl->rdl);

		// Read and feed the lines to the cmdline parser.
		while(fgets(buff, sizeof(buff), fd) )
			cmdline_in(pktgen.cl, buff, strlen(buff));

		fclose(fd);
    }
    return 0;
}

void * pktgen_get_lua() { return pktgen.L; }

#include <poll.h>

/**************************************************************************//**
*
* pktgen_interact - Main interaction routine for command line and display
*
* DESCRIPTION
* Process keyboard input and is called from the main command line loop.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_interact(struct cmdline *cl)
{
	char c;
	struct pollfd	fds;
	uint64_t		curr_tsc;
	uint64_t		next_poll;
	uint64_t		reload;

	fds.fd		= cl->s_in;
	fds.events	= POLLIN;
	fds.revents	= 0;

	c = -1;
	reload = (pktgen.hz/1000);
	next_poll = rte_rdtsc() + reload;

	for(;;) {
		rte_timer_manage();
		curr_tsc = rte_rdtsc();
		if ( unlikely(curr_tsc >= next_poll)  ) {
			next_poll = curr_tsc + reload;
			if ( poll(&fds, 1, 0) ) {
				if ( (fds.revents & (POLLERR | POLLNVAL | POLLHUP)) )
					break;
				if ( (fds.revents & POLLIN) ) {
					if (read(cl->s_in, &c, 1) < 0)
						break;
					if (cmdline_in(cl, &c, 1) < 0)
						break;
				}
			}
		}
	}
}

/**************************************************************************//**
*
* main - Main routine to setup pktgen.
*
* DESCRIPTION
* Main routine to setup pktgen.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

int
main(int argc, char **argv)
{
    int ret, i, idx;
    unsigned lcore_id = 0;

    // call before the rte_eal_init()
    (void)rte_set_application_usage_hook(pktgen_usage);

    memset(&pktgen, 0, sizeof(pktgen));

	// Also scrolls the screen to clear it.
	pktgen.scrn				= scrn_init(MAX_SCRN_ROWS, MAX_SCRN_COLS);

    wr_print_copyright(PKTGEN_APP_NAME, PKTGEN_CREATED_BY);

    pktgen.flags            = PRINT_LABELS_FLAG;
    pktgen.ident            = 0x1234;
    pktgen.nb_rxd           = DEFAULT_RX_DESC;
    pktgen.nb_txd           = DEFAULT_TX_DESC;
    pktgen.nb_ports_per_page= DEFAULT_PORTS_PER_PAGE;
    pktgen.prompt			= "pktgen> ";

    if ( (pktgen.l2p = wr_l2p_create()) == NULL )
    	rte_panic("Unable to create l2p\n");

    /* initialize EAL */
    ret = rte_eal_init(argc, argv);
    if (ret < 0)
        return -1;
    argc -= ret;
    argv += ret;

    pktgen.hz = rte_get_timer_hz();		// Get the starting HZ value.

    rte_delay_ms(100);      // Wait a bit for things to settle.

    /* parse application arguments (after the EAL ones) */
    ret = pktgen_parse_args(argc, argv);
    if (ret < 0)
        return -1;

    /* init drivers */
    if ((ret = rte_pmd_init_all()) < 0)
    	rte_panic("Cannot init devices\n");

    pktgen.portdesc_cnt = wr_get_portdesc(pktgen.portlist, pktgen.portdesc, RTE_MAX_ETHPORTS, 0);

    pktgen.blacklist_cnt = wr_create_blacklist(pktgen.enabled_port_mask, pktgen.portlist,
    		pktgen.portdesc_cnt, pktgen.portdesc);
    scrn_fprintf(0,0, stdout, ">>> Blacklisted port count %d\n", pktgen.blacklist_cnt);

    if ((ret = rte_eal_pci_probe()) < 0)
        rte_panic("Cannot probe PCI, %s\n", rte_strerror(-ret));

    // Open the Lua script handler.
    if ( (pktgen.L = lua_create_instance()) == NULL ) {
		scrn_fprintf(0,0, stdout, "*** Failed to open Lua pktgen support library\n");
    	return -1;
	}

    scrn_fprintf(0, 0, stdout, "\n>>> Packet Burst %d, RX Desc %d, TX Desc %d, mbufs/port %d, mbuf cache %d\n",
    		DEFAULT_PKT_BURST, DEFAULT_RX_DESC,	DEFAULT_TX_DESC, MAX_MBUFS_PER_PORT, MBUF_CACHE_SIZE);

    // Configure and initialize the ports
    pktgen_config_ports();

    printf_info("\n=== Display processing on lcore %d\n", rte_lcore_id());

    /* launch per-lcore init on every lcore except master and master + 1 lcores */
    for (i = 0; i < RTE_MAX_LCORE; i++ ) {
    	if ( (i == rte_get_master_lcore()) || !rte_lcore_is_enabled(i) )
    		continue;
        ret = rte_eal_remote_launch(pktgen_launch_one_lcore, NULL, i);
        if ( ret != 0 )
            scrn_fprintf(0,0, stdout, "Failed to start lcore %d, return %d\n", i, ret);
    }
    rte_delay_ms(500);				// Wait for the lcores to start up.

	// Erase the screen and start updating the screen again.
	scrn_erase();

	wr_logo(3, 16, PKTGEN_APP_NAME);
	wr_splash_screen(3, 16, PKTGEN_APP_NAME, PKTGEN_CREATED_BY);

    scrn_resume();

    pktgen_redisplay(1);

    rte_timer_setup();

    if ( pktgen.flags & ENABLE_GUI_FLAG ) {
        if ( !scrn_is_paused() ) {
            scrn_pause();
            scrn_cls();
            scrn_setw(1);
            scrn_pos(scrn->nrows, 1);
        }

		lua_init_socket(pktgen.L, &pktgen.thread, pktgen.hostname, pktgen.socket_port);
    }

    pktgen_cmdline_start();

    execute_lua_close();
	pktgen_stop_running();

    scrn_pause();

	scrn_setw(1);
	scrn_printf(100, 1, "\n");      // Put the cursor on the last row and do a newline.

    // Wait for all of the cores to stop running and exit.
    rte_eal_mp_wait_lcore();

    return 0;
}

/***********************************************************************//**
*
* pktgen_stop_running - Stop pktgen to exit in a clean way
*
* DESCRIPTION
* Stop all of the logical core threads to stop pktgen cleanly.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void pktgen_stop_running(void)
{
	uint8_t		lid;

	for(lid = 0; lid < RTE_MAX_LCORE; lid++)
		wr_stop_lcore(pktgen.l2p, lid);
}

