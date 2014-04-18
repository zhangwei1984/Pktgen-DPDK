/*
 * SOURCE: pktgen-dump.c
 * LIBS: librte_malloc librte_eal librte_mempool librte_pmd_ring librte_ring
 * LIBS: libethdev
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
