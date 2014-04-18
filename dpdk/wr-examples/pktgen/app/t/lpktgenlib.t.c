/*
 * SOURCE: lpktgenlib.c
 * STUB: pktgen-cmds.h commands.h
 *
 * LIBS: librte_eal libwr_lua librte_mempool librte_cmdline librte_timer
 * LIBS: libwr_common libwr_scrn librte_malloc librte_pmd_ring libethdev
 * LIBS: librte_ring
 * SYSLIBS: pthread m dl
 */


#include "pktgen.h"

pktgen_t pktgen;

const char * help_info[] = {"", "Help info stub", NULL };


// Test driver
int main(void) {
    plan(1);
    ok(1, "ok works");

    done_testing();
    return 0;
}
