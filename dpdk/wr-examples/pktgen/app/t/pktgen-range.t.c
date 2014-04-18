/*
 * SOURCE: pktgen-range.c
 *
 * LIBS: libwr_common libwr_scrn librte_eal librte_mempool librte_malloc
 * LIBS: librte_pmd_ring librte_ring libethdev
 * SYSLIBS: pthread
 */

#include "pktgen.h"

pktgen_t pktgen;


// Test driver
int main(void) {
    plan(1);
    ok(1, "ok works");

    done_testing();
    return 0;
}
