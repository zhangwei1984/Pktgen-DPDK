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
 * 4) The screens displayed by MCOS must contain the copyright notice as defined
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
#include "mcos_main.h"
#include "rte_mcos_debug.h"
#include "commands.h"
#include "phil.h"

mcos_t			mcos;
mcos_master_t * saved_master;

struct lcore_conf {
    volatile uint16_t   stop_running;
    uint16_t			pad0[3];
    void			  * mInfo;
    void			  * arg;
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

	for(i=0; i<mcos.nb_lcores; i++) {
		for(j=0; j<MAX_PHILS; j++) {
			saved[mcos.map_idx2lcores[i]][j].doing = -1;
			saved[mcos.map_idx2lcores[i]][j].count = 0;
		}
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
	scrn_center(1, "%s   %s, %s", msg, mcos_demo_copyright(), POWERED_BY_DPDK);
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
    scrn_printf(last_row, 2, " MCOS Demo %s ", mcos_demo_version());
}


static void
mcos_page_stats(void)
{
	int			row, col, i, j;
	info_t	  * info;
	saved_t	  * save;

    if ( mcos.flags & PRINT_LABELS_FLAG ) {
    	row = 1;
    	col = 1;
    	display_topline("**** Multicore OS ****");
    	scrn_printf(row+1, col, "Duration:");
    	scrn_printf(row+2, col, "Solution:");

   		for(j=0; j<MAX_PHILS; j++) {
   			scrn_printf(j+5, 1, "Philosopher %2d:", j + 1);
   		}

   		col = COLUMN_WIDTH_0;
   		for(i=0; i<mcos.nb_lcores; i++) {
			info = infos[mcos.map_idx2lcores[i]][0];
			if ( info == NULL )
				continue;
			j = i-1;
			scrn_printf(2, col, "%s", info->duration ? "  Fixed" : "  Forever");
			scrn_printf(3, col, "%s", info->solution == CLAIM_BASED ? "  Claim-Based" : "  Ticket-Based");
			scrn_printf(4, col, "  --- lcore %d ---", mcos.map_idx2lcores[i]-1);
			col += COLUMN_WIDTH_1;
   		}
   		mcos.last_row += 2;
   		display_dashline(mcos.last_row);
    }
    mcos.flags &= ~PRINT_LABELS_FLAG;

    col = COLUMN_WIDTH_0;
	for(i=0; i<mcos.nb_lcores; i++) {
		for(j=0; j<MAX_PHILS; j++) {
			info = infos[mcos.map_idx2lcores[i]][j];
			if ( info == NULL )
				continue;
			save = &saved[i][j];

			// Only change the screen if the value has changed.
			if ( save->doing != info->doing ) {
				scrn_printf(j+5, col, "%s", doing_str[info->doing]);
				save->doing = info->doing;
			}
			scrn_printf(j+5, col, "%c", "!@#$%^&*"[save->count]);
		}
		if ( info )
			col += COLUMN_WIDTH_1;
	}
}

void mcos_init_callback( void * mInfo, void * arg )
{
	mcos_config_t		  * cfg = (mcos_config_t *)arg;
	struct lcore_conf	  * qconf = &lcore_conf[rte_lcore_id()];

    // Save the mInfo structure pointer
    qconf->mInfo	= mInfo;
    qconf->arg		= cfg->arg;

	phil_demo_start(mInfo, cfg->arg);
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

    scrn_printf(1, 1, "%c", "-\\|/"[(counter++ & 3)]);

    mcos_page_stats();

    scrn_restore();
}

/* timer1 callback */
static void
mcos_process_stats(__attribute__((unused)) struct rte_timer *tim, __attribute__((unused)) void *arg)
{
	// Maybe add some statistics for the application
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
    int lcore_id = rte_get_master_lcore();

    /* init RTE timer library */
    rte_timer_subsystem_init();

    /* init timer structures */
    rte_timer_init(&timer0);
    rte_timer_init(&timer1);
    rte_timer_init(&timer2);

    /* load timer0, every 1/4 second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer0, (mcos.hz/2), PERIODICAL, lcore_id, mcos_page_display, NULL);

    /* load timer1, every second, on timer lcore, reloaded automatically */
    rte_timer_reset(&timer1, mcos.hz, PERIODICAL, lcore_id, mcos_process_stats, NULL);

    /* load timer2, every 60th of a second, on timer lcore, reloaded automatically */
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

void mcos_quit(void)
{
	phil_quit();
	rte_delay_ms(250);
	cmdline_quit(mcos.cl);
}

void mcos_clear_stats(void)
{
	mcos_redisplay(0);
}

void mcos_show_info(uint16_t lcore)
{
	if ( (lcore < mcos.nb_lcores) && lcore_conf[mcos.map_idx2lcores[lcore]].mInfo )
		mcos_info(lcore_conf[mcos.map_idx2lcores[lcore]].mInfo);
}

void mcos_show_ready(uint16_t lcore)
{
	mInfo_t	  * mInfo;

	if ( (lcore < mcos.nb_lcores) && lcore_conf[mcos.map_idx2lcores[lcore]].mInfo ) {
		mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo;
		mcos_listq(mInfo, "ready", mInfo->rdyhead);
	}
}

void mcos_show_sleep(uint16_t lcore)
{
	mInfo_t	  * mInfo;

	if ( (lcore < mcos.nb_lcores) && lcore_conf[mcos.map_idx2lcores[lcore]].mInfo ) {
		mInfo = lcore_conf[mcos.map_idx2lcores[lcore]].mInfo;
		mcos_listq(mInfo, "sleepq", mInfo->sleepq);
	}
}

void mcos_show_fibers(uint16_t lcore)
{
	if ( (lcore > mcos.nb_lcores) || (lcore_conf[mcos.map_idx2lcores[lcore]].mInfo == NULL))
		return;

	mcos_list_fibers(lcore_conf[mcos.map_idx2lcores[lcore]].mInfo);
}

void mcos_show_stackcheck(uint16_t lcore)
{
	if ( (lcore > mcos.nb_lcores) || (lcore_conf[mcos.map_idx2lcores[lcore]].mInfo == NULL))
		return;

	mcos_stack_check(lcore_conf[mcos.map_idx2lcores[lcore]].mInfo);
}

void mcos_list_master(void)
{
	mcos_master_list(saved_master);
}

void mcos_show_instances(uint16_t idx)
{
	mcos_list_instance_id(saved_master, idx);
}

/* display usage */
static void
mcos_usage(const char *prgname)
{
    printf("%s [EAL options] -- [-h]\n"
           "  -h           Display the help information\n", prgname);
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
	uint32_t			poll_rate;
	struct lcore_conf * qconf = &lcore_conf[rte_lcore_id()];

	fds.fd		= cl->s_in;
	fds.events	= POLLIN;
	fds.revents	= 0;
	poll_rate	= 1000;

	rte_delay_ms(1000);

	c = -1;
	for(;;) {
        if ( unlikely(qconf->stop_running > 0) )
            break;
        rte_timer_manage();

		if ( --poll_rate )
			continue;
		poll_rate = 1000;

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
    printf("Found %d worker lcores found, does not include master lcore\n", mcos.nb_lcores);

    mcos_clear_saved();

    mcos.hz = rte_get_timer_hz();

    mcos.last_row = 20;

    // Allocate the memory for the MCOS master if needed.
    master = mcos_master_initialize();
    if ( master == NULL )
    	rte_panic("Master MCOS could not be initialized");

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
    RTE_LCORE_FOREACH(lcore_id) {
    	mcos_stop( lcore_conf[lcore_id].mInfo );
        if (rte_eal_wait_lcore(lcore_id) < 0)
            return -1;
    }

    return 0;
}
