/*
 * SOURCE: commands.c
 * STUB: pktgen-cmds.h pktgen-main.h pktgen-capture.h lpktgenlib.h
 *
 * LIBS: libwr_scrn librte_eal librte_mempool librte_timer librte_cmdline
 * LIBS: libwr_common libwr_lua librte_pmd_ring libethdev librte_malloc
 * LIBS: librte_ring
 * SYSLIBS: pthread m
 */

#include "pktgen.h"
#include "lpktgenlib.h"


pktgen_t pktgen;


// Test driver
int main(void) {
    plan(1);
    ok(1, "ok works");

    done_testing();
    return 0;
}
