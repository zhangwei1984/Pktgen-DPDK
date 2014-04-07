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

#ifndef _EXTRA_ETHERTYPES_H_
#define _EXTRA_ETHERTYPES_H_

/******************************************************************************
 * MPLS
 ******************************************************************************
 * An MPLS label has the following layout:
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * |                 Label                 | EXP |S|     TTL
 *
 * Label: 20-bit label value
 * EXP: Experimental (QoS and ECN)
 *   - 3-bit Traffic Class field for QoS priority
 *   - Explicit Congestion Notification
 * S: Bottom-of-Stack. If set, the current label is the last in the stack.
 * TTL: Time-to-Live
 */

/* These EtherTypes are not declared in rte_ether.h */
#define ETHER_TYPE_MPLS_UNICAST		0x8847
#define ETHER_TYPE_MPLS_MULTICAST	0x8848

struct mpls_hdr {
	uint32_t label;				/**< MPLS label */
} __attribute__ ((__packed__));

/* Set/clear Bottom of Stack flag */
#define MPLS_SET_BOS(mpls_label) do { mpls_label |=  (1 << 8); } while (0);
#define MPLS_CLR_BOS(mpls_label) do { mpls_label &= ~(1 << 8); } while (0);


/******************************************************************************
 * Q-in-Q (802.1ad)
 ******************************************************************************
 * An 802.1ad header consists of 2 consecutive VLAN headers. A VLAN header has
 * the following layout:
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * |               TPID            | PCP |D|         VID
 *
 * TPID: Tag protocol identifier. The EtherType associated with the tag.
 *   - the first tag: 0x88A8
 *   - the second tag: 0x8100 (VLAN EtherType)
 * PCP: Priority code point. Class of Service indicator
 * D: Drop eligible indicator
 * VID: VLAN identifier
 */

/* These EtherTypes are not declared in rte_ether.h */
#define ETHER_TYPE_Q_IN_Q	0x88A8

struct qinq_hdr {
	uint16_t qinq_tci;		/**< Outer tag PCP, DEI, VID */
	uint16_t vlan_tpid;		/**< Must be ETHER_TYPE_VLAN (0x8100) */
	uint16_t vlan_tci;		/**< Inner tag PCP, DEI, VID */
	uint16_t eth_proto;		/**< EtherType of encapsulated frame */
} __attribute__ ((__packed__));


/******************************************************************************
 * Generic Routing Encapsulation (GRE)
 ******************************************************************************
 * A GRE packet header has the following structure:
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * |C| |K|S|    Reserved0    | Ver |         Protocol Type         |
 * |      Checksum (optional)      |     Reserved1 (optional)      |
 * |                         Key (optional)                        |
 * |                  Sequence Number (optional)                   |
 *
 * C: 1 if the Checksum and the Reserved1 fields are present and the Checksum
 *    field contains valid information
 * K: 1 if the Key field is present
 * S: the Sequence Number field is present
 * Reserved0: must be 0
 * Ver: Version Number, must be 0
 * Protocol Type: EtherType of encapsulated packet. When IPv4 is being carried
 *   as the GRE payload, the Protocol Type field MUST be set to 0x800.
 * Checksum: the IP (one's complement) checksum sum of all the 16 bit words in
 *   the GRE header and the payload packet
 * Reserved1: must be 0
 * Key: 32bit number determined by the encapsulator
 * Sequence Number: for packet ordering purposes
 */

/* ipHdr_t.proto value in the IP Header.
 * 47	GRE		Generic Routing Encapsulation			[RFC 2784, RFC 2890]
 *
 * (IPPROTO_* is #define'd in librte_net/rte_ip.h)
 */
#define PG_IPPROTO_GRE		IPPROTO_GRE


/* GRE header */
typedef struct greHdr_s {
	uint8_t		reserved0_0 : 4;	/**< must be 0 */
	uint8_t		seq_present : 1;	/**< Sequence Number present */
	uint8_t		key_present : 1;	/**< Key present */
	uint8_t		unused      : 1;
	uint8_t		chk_present : 1;	/**< Checksum and Reserved1 present */
	uint8_t		version     : 3;	/**< Version Number */
	uint8_t		reserved0_1 : 5;	/**< must be 0 */
	uint16_t	eth_type;			/**< EtherType of encapsulated packet */
	uint32_t	extra_fields[3];	/**< Room for Checksum+Reserved1, Key and Sequence Number fields if present */
} __attribute__ ((__packed__)) greHdr_t;

/* GRE/IPv4 pseudo header */
typedef struct greIp_s {
	ipHdr_t		ip;		/* Outer IPv4 header */
	greHdr_t	gre;	/* GRE header for protocol */
} __attribute__ ((__packed__)) greIp_t;


/******************************************************************************
 * Address Resolution Protocol (ARP)
 ******************************************************************************
 * An ARP packet has the following structure:
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * |             HTYPE             |             PTYPE             |
 * |       HLEN    |      PLEN     |         Operation (OPER)      |
 * |         Sender Hardware Address (SHA)         | Sender Proto-
 * |               col Address (SPA)               | Target Hard-
 * |      ware Address (THA)       | Target Protocol Address (TPA)
 * |                               ||
 *
 * HTYPE: Hardware type. Ethernet is 1.
 * PTYPE: Protocol type. For IPv4, this has the value 0x0800. The permitted
 *    PTYPE values share a numbering space with those for EtherType.
 * HLEN: Hardware length. Length (in octets) of a hardware address. Ethernet
 *    addresses size is 6.
 * PLEN: Protocol length. Length (in octets) of addresses used in the upper
 *    layer protocol, specified in PTYPE. IPv4 address size is 4.
 * OPER: Operation that the sender is performing: 1 for request, 2 for reply.
 * SHA: MAC address of the sender.
 * SPA: IP address of the sender.
 * THA: MAC address of the intended receiver.
 * TPA: IP address of the intended receiver.
 */

/* ARP header */
typedef struct arp_hdr_s {
	uint16_t	htype;		/**< Hardware type */
	uint16_t	ptype;		/**< Protocol type */
	uint8_t		hlen;		/**< Hardware length */
	uint8_t		plen;		/**< Protocol length */
	uint16_t	oper;		/**< Operation */
	struct ether_addr sha;	/**< Sender Hardware Address */
	uint32_t	spa;		/**< Sender Protocol Address */
	struct ether_addr tha;	/**< Target Hardware Address */
	uint32_t	tpa;		/**< Target Protocol Address */
} __attribute__ ((__packed__)) arp_hdr_t;


#endif /* _EXTRA_ETHERTYPES_H_ */
