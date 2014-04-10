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

// Allocated the pktgen structure for global use
extern    pktgen_t        pktgen;

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

void
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
