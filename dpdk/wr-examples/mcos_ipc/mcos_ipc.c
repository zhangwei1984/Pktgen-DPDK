/**
 * <COPYRIGHT_TAG>
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
/* Created 2014 by Keith Wiles @ windriver.com */

#include <rte_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <termios.h>

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
#include <rte_random.h>
#include <rte_timer.h>
#include <rte_debug.h>
#include <rte_ring.h>
#include <rte_msg.h>
#include <rte_mempool.h>
#include <rte_spinlock.h>
#include <rte_version.h>

#include <cmdline_rdline.h>
#include <cmdline_parse.h>
#include <cmdline_socket.h>
#include <cmdline_parse_string.h>
#include <cmdline_parse_num.h>
#include <cmdline.h>

#include "rte_mcos.h"
#include "mcos_ipc.h"
#include "rte_mcos_debug.h"
#include "commands.h"

mcos_t			mcos;
mcos_master_t * saved_master;
uint16_t		syncSema;
uint16_t		fiberCnt;

typedef struct stats_s {
	uint64_t			rx_cnt;
	uint64_t			tx_cnt;
	uint64_t			rx_sav;
	uint64_t			tx_sav;
	uint64_t			rx_mps;
	uint64_t			tx_mps;
} stats_t;

#define RX_DATA_COUNT	RTE_FIBER_MSG_COUNT
#define TX_DATA_COUNT	RTE_INSTANCE_MSG_COUNT
typedef struct fiber_s {
    uint16_t			pid;
    uint16_t			ready;
    uint16_t			tx_idx;
    int32_t				pad0;
    mcos_data_t			rx_data[RX_DATA_COUNT+1];
    mcos_data_t			tx_data[TX_DATA_COUNT+1];
    stats_t				s;
} fiber_t;

struct lcore_conf {
    volatile uint16_t   stop_running;
    uint16_t			tx_enabled;
    uint16_t			pad0[2];
    void			  * mInfo;
    void			  * arg;
    fiber_t				fibers[MAX_DEMO_THREADS];
    uint64_t			clock_intr_count;
} __rte_cache_aligned;

struct lcore_conf lcore_conf[RTE_MAX_LCORE];

static __inline__ void mcos_stop_running(void) {
	unsigned		i;

	for(i = 0; i < mcos.nb_lcores; i++)
		lcore_conf[mcos.map_idx2lcores[i]].stop_running = 1;
}

static __inline__ void mcos_clear_saved(void) {
	int		i, j;

	if ( saved_master )
		saved_master->dispatch_error = saved_master->dispatch_sent = 0;

	for(i=0; i<mcos.nb_lcores; i++) {
		for(j=0; j < MAX_DEMO_THREADS; j++)
			memset(&lcore_conf[mcos.map_idx2lcores[i]].fibers[j].s, 0, sizeof(stats_t));
	}
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
	scrn_center(1, "%s   %s, %s", msg, mcos_ipc_copyright(), POWERED_BY_DPDK);
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
    for(i=0; i<(mcos.scrn->ncols-15); i++)
    	printf_info("-");
    scrn_printf(last_row, 2, " MCOS IPC %s ", mcos_ipc_version());
}

static void
mcos_page_stats(void)
{
	int			row, col, i, j;
	struct lcore_conf	  * qconf;
	char		buff[32];

    if ( mcos.flags & PRINT_LABELS_FLAG ) {
    	row = 1;
    	col = 1;
    	display_topline("**** MCOS IPC ****");

    	row += 3;
    	qconf = &lcore_conf[1];
   		for(j=0; j<MAX_DEMO_THREADS; j++) {
   			scrn_printf(j+row, 1, "Fiber %2d    %04x:", j, qconf->fibers[j].pid);
   		}
   		scrn_printf(row+j, 1, "Dispatch Err/Cnt:");

   		col = COLUMN_WIDTH_0;
   		for(i=0; i<mcos.nb_lcores; i++) {
			scrn_printf(2, col, "  --- lcore %d ---", i);
			scrn_printf(3, col, "      Tx/Rx mps   ");
			col += COLUMN_WIDTH_1;
   		}

   		mcos.last_row += 2;
   		display_dashline(mcos.last_row);
    }
    mcos.flags &= ~PRINT_LABELS_FLAG;

    col = COLUMN_WIDTH_0;
	for(i=0; i<mcos.nb_lcores; i++) {
    	qconf = &lcore_conf[mcos.map_idx2lcores[i]];
		for(j=0; j<MAX_DEMO_THREADS; j++) {
			snprintf(buff, sizeof(buff), "%ld/%ld", qconf->fibers[j].s.tx_mps, qconf->fibers[j].s.rx_mps);
			scrn_printf(row+j, col, "  %16s", buff);
		}
		col += COLUMN_WIDTH_1;
	}
    col = COLUMN_WIDTH_0;
	snprintf(buff, sizeof(buff), "%ld/%ld", saved_master->dispatch_error, saved_master->dispatch_sent);
	scrn_printf(row+16, col, "  %16s", buff);
}

static void msg_fiber(mInfo_t * mInfo, void * arg)
{
	uint16_t				lcore = rte_lcore_id(), idx;
	struct lcore_conf	  * qconf = &lcore_conf[lcore];
	uint64_t				info;
	int32_t					rx_cnt, tx_cnt;
	uint64_t				t = (uint64_t)arg;
	mcos_addr_t				dst_id;
	fiber_t				  * fiber;

	mcos_msg_create(mInfo, RTE_FIBER_MSG_COUNT);

	qconf->fibers[t].ready = 1;
	mcos_wait(mInfo, syncSema);

	info = -1;

	idx = (lcore - mcos.master_lcore) - 1;
	if ( idx < (mcos.nb_lcores - 1) )
		dst_id = mcos_mk_instance(idx+1, mcos.map_idx2lcores[idx+1], mcos_currpid(mInfo));
	else
		dst_id = mcos_mk_instance(0, mcos.map_idx2lcores[0], mcos_currpid(mInfo));
	while( qconf->stop_running == 0 ) {
		// Do not run the fiber if 't' is equal to fiberCnt
		if ( unlikely(t >= fiberCnt) ) {
			mcos_delay_ms(mInfo, 250);
			continue;
		}

		fiber = &qconf->fibers[t];
		if ( likely(qconf->tx_enabled) ) {
			mcos_data_t	  * d = &fiber->tx_data[fiber->tx_idx++];

			d->dst_id.data	= 0;
			d->src_id.data	= 0;
			d->info			= info;

			if ( unlikely(fiber->tx_idx == TX_DATA_COUNT) ) {
				fiber->tx_idx = 0;

				tx_cnt = mcos_send_message_burst(mInfo, dst_id, &fiber->tx_data[0], TX_DATA_COUNT);
				fiber->s.tx_cnt += tx_cnt;
			}
		}

		rx_cnt = mcos_recv_message_burst(mInfo, &fiber->rx_data[0], RX_DATA_COUNT);
		if ( likely(rx_cnt > 0) )
			fiber->s.rx_cnt += rx_cnt;
		else
			mcos_yield(mInfo);			// Let some other fiber run.
	}
}

static void msg_fiber_start( mInfo_t * mInfo )
{
	struct lcore_conf	  * qconf = &lcore_conf[rte_lcore_id()];
	int64_t		i, notReady;
	char		name[24];

	syncSema = mcos_screate(mInfo, 0);

	for( i = 0; i<MAX_DEMO_THREADS; i++) {
		memset(&qconf->fibers[i], 0, sizeof(fiber_t));
		snprintf(name, sizeof(name), "Fiber-%ld", i);
		qconf->fibers[i].pid = mcos_spawn(mInfo, name, 200, 0, (func_t)msg_fiber, (void *)i);
		mcos_resume(mInfo, qconf->fibers[i].pid);
	}

	// Wait for all of the fibers to become ready.
	notReady = 1;
	while( notReady ) {
		notReady = 0;
		mcos_delay_ms(mInfo, 100);
		for(i=0; i< MAX_DEMO_THREADS; i++) {
			if ( lcore_conf[rte_lcore_id()].fibers[i].ready == 0 )
				notReady = 1;
		}
	}
	// Kick everyone off the semaphore.
	mcos_semflush(mInfo, syncSema);

	// Exit and terminate the StartFiber process
}

static void mcos_demo_start( mInfo_t * mInfo, void * arg )
{
	char		name[24];

	snprintf(name, sizeof(name), "StartFiber");
	mcos_resume(mInfo, mcos_spawn(mInfo, name, 400, 0, (func_t)msg_fiber_start, arg));
	// StartFiber will exit after it finishes initializing the other fibers.
}

void mcos_init_callback( void * mInfo, void * arg )
{
	mcos_config_t	  * cfg = (mcos_config_t *)arg;
	struct lcore_conf * qconf = &lcore_conf[rte_lcore_id()];

    // Save the mInfo structure pointer
    qconf->mInfo	= mInfo;
    qconf->arg		= cfg->arg;

	mcos_demo_start(mInfo, cfg->arg);
}

/* timer0 callback to process screen updates */
static void
mcos_page_display(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
    static unsigned int counter = 0;

    // Leave if the screen is paused
    if ( scrn_is_paused() )
        return;

    scrn_save();

    scrn_printf(1,1, "%c", "-\\|/"[(counter++ & 3)]);

    mcos_page_stats();

    scrn_restore();
}

/* timer1 callback */
static void
mcos_process_stats(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
	int32_t		i, j;
	fiber_t	  * f;

	for(i = 0; i<mcos.nb_lcores; i++) {
		f = &lcore_conf[mcos.map_idx2lcores[i]].fibers[0];
		for(j = 0; j<MAX_DEMO_THREADS; j++, f++) {
			f->s.rx_mps = f->s.rx_cnt - f->s.rx_sav;
			f->s.tx_mps = f->s.tx_cnt - f->s.tx_sav;
			f->s.rx_sav = f->s.rx_cnt;
			f->s.tx_sav = f->s.tx_cnt;
		}
	}
}

/* timer1 callback */
static void
mcos_process_tick(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
	int			i;

	for( i = 0; i < mcos.nb_lcores; i++) {
		if ( lcore_conf[mcos.map_idx2lcores[i]].mInfo ) {
			lcore_conf[mcos.map_idx2lcores[i]].clock_intr_count++;
			mcos_interrupt(lcore_conf[mcos.map_idx2lcores[i]].mInfo, CLOCK_VECTOR, i);
		}
	}
}

static struct rte_timer timer0;
static struct rte_timer timer1;
static struct rte_timer timer2;

static void
mcos_timer_setup(void)
{
    int lcore_id = mcos.master_lcore;

    /* init RTE timer library */
    rte_timer_subsystem_init();

    /* init timer structures */
    rte_timer_init(&timer0);
    rte_timer_init(&timer1);
    rte_timer_init(&timer2);

    /* load timer0, every 1/4 second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer0, (mcos.hz/4), PERIODICAL, lcore_id, mcos_page_display, NULL);

    /* load timer1, every second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer1, mcos.hz, PERIODICAL, lcore_id, mcos_process_stats, NULL);

    /* load timer2, every Nth of a second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer2, mcos.hz/mcos_get_tickrate(), PERIODICAL, lcore_id, mcos_process_tick, NULL);
}

void
mcos_redisplay( int cls_flag )
{
	scrn_pause();
	if ( cls_flag ) {
		scrn_cls();
		scrn_pos(100, 1);
	}
	mcos.flags |= PRINT_LABELS_FLAG;

	mcos_clear_saved();
	scrn_resume();

	mcos_page_display(NULL, NULL);
}

// ===============================================================================
// Commands used at the prompt.
void mcos_quit(void)
{
	rte_delay_ms(250);
	cmdline_quit(mcos.cl);
}

void mcos_clear_stats(void)
{
	mcos_clear_saved();
	mcos_redisplay(0);
}

void mcos_show_info(uint16_t lcore)
{
	mInfo_t		* mInfo;

	if ( lcore < mcos.nb_lcores && (mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo) )
		mcos_info(mInfo);
}

void mcos_show_ready(uint16_t lcore)
{
	mInfo_t		* mInfo;

	if ( lcore < mcos.nb_lcores && (mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo) )
		mcos_listq(mInfo, "ready", mInfo->rdyhead);
}

void mcos_show_sleep(uint16_t lcore)
{
	mInfo_t	  * mInfo;

	if ( lcore < mcos.nb_lcores && (mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo) )
		mcos_listq(mInfo, "sleepq", mInfo->sleepq);
}

void mcos_show_fibers(uint16_t lcore)
{
	mInfo_t	  * mInfo;

	if ( lcore < mcos.nb_lcores && (mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo) )
		mcos_list_fibers(mInfo);
}

void mcos_start_sending(uint32_t bitmap)
{
	int32_t		lcore;

	for( lcore=0; lcore<mcos.nb_lcores; lcore++) {
		if ( bitmap & (1 << lcore) )
			lcore_conf[mcos.map_idx2lcores[lcore]].tx_enabled = 1;
	}
}

void mcos_stop_sending(uint32_t bitmap)
{
	int32_t		lcore;

	for( lcore=0; lcore<mcos.nb_lcores; lcore++) {
		if ( bitmap & (1 << lcore) )
			lcore_conf[mcos.map_idx2lcores[lcore]].tx_enabled = 0;
	}
}

void mcos_list_master(void)
{
	if( saved_master )
		mcos_master_list(saved_master);
}

void mcos_show_instances(uint16_t idx)
{
	if ( saved_master )
		mcos_list_instance_id(saved_master, idx);
}

void mcos_show_stackcheck(uint16_t lcore)
{
	if ( lcore >= mcos.nb_lcores || (lcore_conf[mcos.map_idx2lcores[lcore]].mInfo == NULL))
		return;

	mcos_stack_check(lcore_conf[mcos.map_idx2lcores[lcore]].mInfo);
}

void mcos_set_fiber_count( uint16_t value )
{
	fiberCnt = value;
}

void mcos_show_ring(uint16_t lcore, uint16_t pid)
{
	mInfo_t	  * mInfo;

	if ( lcore >= mcos.nb_lcores || ((mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo) == NULL))
		return;

	if ( mInfo->fcbs[pid].msg_ring)
		rte_msg_dump(mInfo->fcbs[pid].msg_ring);
}

/**************************************************************************//**
*
* mcos_screen - Enable or Disable screen updates.
*
* DESCRIPTION
* Enable or disable screen updates.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_screen(const char * onOff)
{
	if ( parseState(onOff) == DISABLE_STATE ) {
		scrn_pause();
		scrn_cls();
		scrn_setw(1);
		scrn_pos(100, 1);
	} else {
		scrn_cls();
		scrn_pos(100,1);
		scrn_setw(mcos.last_row+1);
		scrn_resume();
		mcos_redisplay(1);
	}
}


// ===============================================================================

/* display usage */
static void
mcos_usage(const char *prgname)
{
    printf("%s [EAL options] -- [-h]\n"
           "  -h           Display the help information\n",
           prgname);
}

/* Parse the argument given in the command line of the application */
static int
mcos_parse_args(int argc, char **argv)
{
    int opt, ret;
    char **argvopt;
    int option_index;
    char *prgname = argv[0];
    static struct option lgopts[] = {
        {NULL, 0, 0, 0}
    };

    argvopt = argv;

    while ((opt = getopt_long(argc, argvopt, "h",
                  lgopts, &option_index)) != EOF) {
        switch (opt) {
        case 'h':
            mcos_usage(prgname);
            return -1;

        /* long options */
        case 0:
            mcos_usage(prgname);
            return -1;

        default:
            mcos_usage(prgname);
            return -1;
        }
    }

    if (optind >= 0)
        argv[optind-1] = prgname;

    ret = optind-1;
    optind = 0;				/* reset getopt lib */
    return ret;
}

#include <poll.h>

static void
mcos_poll_interact(struct cmdline *cl)
{
	char 				c;
	struct pollfd		fds;
	struct lcore_conf * qconf = &lcore_conf[rte_lcore_id()];

	fds.fd		= cl->s_in;
	fds.events	= POLLIN;
	fds.revents	= 0;

	rte_delay_ms(1000);

	for(;;) {
        if ( unlikely(qconf->stop_running > 0) )
            break;
        rte_timer_manage();
		mcos_msg_dispatch(saved_master);

		if ( poll(&fds, 1, 0) ) {
			if ( (fds.revents & (POLLERR | POLLNVAL | POLLHUP)) )
				break;
			c = -1;
			if ( (fds.revents & POLLIN) ) {
				if (read(cl->s_in, &c, 1) < 0)
					break;
				if (cmdline_in(cl, &c, 1) < 0)
					break;
			}
		}
	}
}

int
main(int argc, char ** argv)
{
	mcos_master_t * master;
	mcos_config_t   config;
    uint32_t		i;
    int32_t			ret;
    unsigned int	lcore_id = 0;

    mcos_dbgInit(INCLUDE_LOG_SUPPORT);

    (void)rte_set_application_usage_hook(mcos_usage);

    /* init EAL */
    ret = rte_eal_init(argc, argv);
    if (ret < 0)
        return -1;
    argc -= ret;
    argv += ret;

	// Also scrolls the screen to clear it.
	mcos.scrn = scrn_init(MAX_SCRN_ROWS, MAX_SCRN_COLS);

    /* parse application arguments (after the EAL ones) */
    ret = mcos_parse_args(argc, argv);
    if (ret < 0)
        return -1;

    // Find the first and last lcore configured by the system
    mcos.nb_lcores = mcos_map_lcores(mcos.map_idx2lcores, &mcos.master_lcore);
    printf("Found %d lcores to use\n", mcos.nb_lcores);

    mcos_clear_saved();

    mcos.hz = rte_get_timer_hz();

    mcos.last_row = 21;

    master = mcos_master_initialize();
    if ( master == NULL )
    	rte_panic("Master MCOS could not be initialized");

    fiberCnt			= MAX_DEMO_THREADS;
    saved_master		= master;
    config.master		= master;
    config.num_fibers	= DEFAULT_NUM_FIBERS;
    config.num_semas	= DEFAULT_NUM_SEMAS;
    config.num_mutexs	= DEFAULT_NUM_MUTEXS;

    // Launch the other MCOS based lcores.
    for( i = 0; i < mcos.nb_lcores; i++) {
    	config.arg		= &lcore_conf[mcos.map_idx2lcores[i]];

    	mcos_remote_launch(master, &config, mcos.map_idx2lcores[i]);
    }

	mcos.flags |= PRINT_LABELS_FLAG;

	scrn_erase();
	scrn_resume();
	mcos_redisplay(0);

    mcos_timer_setup();

	// Start up the command line, which exits on Control-D
	mcos.cl = cmdline_stdin_new(main_ctx, "mcos> ");
	if ( mcos.cl ) {
		mcos_poll_interact(mcos.cl);
		cmdline_stdin_exit(mcos.cl);
	}
	mcos_stop_running();

	scrn_setw(1);
	scrn_printf(100, 1, "\n");      // Put the cursor on the last row and do a newline.

	// Wait for all of the cores to stop running and exit.
    RTE_LCORE_FOREACH_SLAVE(lcore_id) {
    	mcos_stop( lcore_conf[lcore_id].mInfo );
        if (rte_eal_wait_lcore(lcore_id) < 0)
            return -1;
    }

    return 0;
}
