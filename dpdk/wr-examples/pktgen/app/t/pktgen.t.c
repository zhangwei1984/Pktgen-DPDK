/*
 * SOURCE: pktgen.c
 * STUB: rte_cycles.h rte_eal.h rte_scrn.h wr_copyright_info.h pktgen-random.h
 * STUB: pktgen-ether.h pktgen-gre.h pktgen-tcp.h pktgen-ipv4.h pktgen-udp.h
 * STUB: wr_cksum.h pktgen-ipv6.h pktgen-arp.h pktgen-vlan.h pktgen-range.h
 * STUB: pktgen-dump.h pktgen-capture.h pktgen-cpu.h pktgen-pcap.h pktgen-seq.h
 * STUB: pktgen-stats.h rte_timer.h
 */


/*
 * rte_ethdev.h fake functions
 */
void rte_eth_dev_stop(uint8_t port_id) { return; }
int rte_eth_dev_start(uint8_t port_id) { return 0; }


int rte_cycles_vmware_tsc_map;
__thread unsigned per_lcore__lcore_id;
struct rte_eth_dev rte_eth_devices[RTE_MAX_ETHPORTS];


// Test driver
int main(void) {
    plan(1);
    ok(1, "ok works");

    done_testing();
    return 0;
}
