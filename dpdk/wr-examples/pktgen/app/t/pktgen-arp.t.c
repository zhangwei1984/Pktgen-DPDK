/*
 * SOURCE: pktgen-arp.c
 * LIBS:
 */

#include "pktgen.h"

pktgen_t pktgen;


// Function stubs
STUB_VOID(scrn_fprintf, int16_t r, int16_t c, FILE * f, const char * fmt, ...);
STUB_VOID(pktgen_send_mbuf, struct rte_mbuf *m, uint8_t pid, uint8_t qid);

STUB_NULL(pkt_seq_t *, pktgen_find_matching_ipdst, port_info_t * info, uint32_t addr);
STUB_NULL(pkt_seq_t *, pktgen_find_matching_ipsrc, port_info_t * info, uint32_t addr);

STUB_VOID(pktgen_packet_ctor, port_info_t * info, int32_t seq_idx, int32_t type);
STUB_VOID(pktgen_redisplay, int cls_flag );

// Variable stubs
__thread unsigned per_lcore__lcore_id;


// Test driver
int main(void) {
    plan(1);
    ok(1, "ok works");

    done_testing();
    return 0;
}
