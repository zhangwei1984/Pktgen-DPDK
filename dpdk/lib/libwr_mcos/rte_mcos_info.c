/**
 * <COPYRIGHT_TAG>
 */
/**
 * Copyright (c) <2010-2012>, Wind River Systems, Inc.
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
/* Created 2012 by Keith Wiles @ windriver.com */

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

#define __USE_GNU
#include "rte_mcos.h"

/**************************************************************************//**
*
* mcos_info - Dump out some information about mcos and its fibers
*
* DESCRIPTION
* Dump out some information about mcos to the console.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_info(mInfo_t * mInfo)
{
	printf("\n*** Multicore OS Version %s (mInfo %p) ***\n", mcos_version(), mInfo);
	if ( (mInfo == NULL) || ((mInfo->flags & INIT_FLAG) == 0) ) {
		printf("--- MCOS is not initialized!\n");
		return;
	}

	printf("  Current PID     : %4d (%s)\n", mInfo->currpid, mInfo->curr->name);
	printf("  Number of Fibers: %4d,  Queues: %4d,  Semaphores: %4d,  Mutexs: %4d\n\n",
			mInfo->num_fibers, mInfo->num_queues, mInfo->num_semas, mInfo->num_mutexs);

	printf("  Structure sizes (in bytes):  mcos_master_t: %5lu + RX data (%d * sizeof(mcos_data_t)): %6lu\n",
			sizeof(mcos_master_t), mInfo->master->msg_cnt, (mInfo->master->msg_cnt * sizeof(mcos_data_t)));
	printf("          mInfo_t: %3lu     fcb_t: %3lu  mcos_instance_t: %3lu  mentry_t: %3lu  mcos_data_t: %3lu\n",
			sizeof(mInfo_t), sizeof(fcb_t), sizeof(mcos_instance_t), sizeof(mentry_t), sizeof(mcos_data_t));
	printf("         qentry_t: %3lu  sentry_t: %3lu           data_u: %3lu    fifo_t: %3lu\n",
			sizeof(qentry_t), sizeof(sentry_t), sizeof(data_u), sizeof(fifo_t));

	printf("\n");
	printf("  Memory allocated (in bytes):\n");
	printf("       Fibers (fcb_t)        : %6lu\n", (mInfo->num_fibers * sizeof(fcb_t)) );
	printf("       Queues (qentry_t)     : %6lu\n", (mInfo->num_queues * sizeof(qentry_t)) );
	printf("       Semaphores (sentry_t) : %6lu\n", (mInfo->num_semas * sizeof(sentry_t)) );
	printf("       Mutexs (mentry_t)     : %6lu\n", (mInfo->num_mutexs * sizeof(mentry_t)) );
	printf("       Master (mcos_master_t): %6lu\n", sizeof(mcos_master_t) );
	printf("       Master RX data        : %6lu = %lu Total bytes\n\n",
			(mInfo->master->msg_cnt * sizeof(mcos_data_t)),
			((mInfo->num_fibers * sizeof(fcb_t)) +
			(mInfo->num_queues * sizeof(qentry_t)) + sizeof(mInfo_t) +
			(mInfo->num_mutexs * sizeof(mentry_t)) +
			(mInfo->num_semas * sizeof(sentry_t)) +
			(mInfo->master->msg_cnt * sizeof(mcos_data_t)) +
			sizeof(mcos_master_t)));
	printf("  Ready Queue: Head %d, Tail %d, ", mInfo->rdyhead, mInfo->rdytail);
	printf("  Clock Queue: Head %d\n", mInfo->sleepq);
	printf("  Schedule count %lu, clock ticks %lu, tickrate %d, slnempty %d, clkdiff %d\n",
			mInfo->scheduler_cnt, mInfo->clk, mInfo->tickrate, mInfo->slnempty, mInfo->clkdiff);
}

/**************************************************************************//**
*
* mcos_listq - Dump out a list or queue
*
* DESCRIPTION
* Dump out a list or queue of information.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_listq(mInfo_t * mInfo, c8_t * msg, int16_t head)
{
	qentry_t  * q = &mInfo->q[0];
	int16_t		tail = head + 1;
	int			i, k;

	printf("\n*** %s queue:\n", msg);

	printf("       Pid:  Next  Prev    Key Name               State\n");
	printf("head %5d: %5d %5d %6d\n", head, q[head].next, q[head].prev, q[head].key);
	for(i = q[head].next, k=0; (i != tail) && (k < 8); i = q[i].next, k++) {
		printf("     %5d: %5d %5d %6d (%-16s) %s\n", i, q[i].next, q[i].prev,
				q[i].key, mInfo->fcbs[i].name, mcos_state(mInfo->fcbs[i].state));
	}
	printf("tail %5d: %5d %5d %6d\n", tail, q[tail].next, q[tail].prev, q[tail].key);
	tail = mInfo->currpid;
	printf("curr %5d: %5d %5d %6d (%-16s) %s\n\n", tail, q[tail].next, q[tail].prev,
			q[tail].key, mInfo->fcbs[tail].name, mcos_state(mInfo->fcbs[tail].state) );
}

/**************************************************************************//**
*
* mcos_master_list - Dump out the master list
*
* DESCRIPTION
* List the Master information instance data.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_master_list( mcos_master_t * master )
{
	mInfo_t	  * mInfo;
	mcos_addr_t	addr;
	int32_t		i;

	printf("Master pointer: %p\n", master);
	printf("Instance ID            mInfo           master             ring  Name\n");
	for(i = 0; i < MAX_NUM_MCOS; i++) {
		mInfo = mcos_get_instance_info(&master->instance[i]);
		if ( mInfo == NULL )
			continue;

		addr = mInfo->instance->instance_id;
		printf(" %02x:%02x:%04x %16p %16p %16p  %s\n",
				mcos_instance_index(addr),
				mcos_instance_lcore(addr),
				mcos_instance_pid(addr),
				mInfo, mInfo->instance->master, mInfo->instance->msg_ring,
				mInfo->instance->name);
	}
}

/**************************************************************************//**
*
* mcos_list_instance_id - List all instances and their names
*
* DESCRIPTION
* Find a instance id matching the given name.
*
* RETURNS: E_ERROR or instance_id
*
* SEE ALSO:
*/

uint32_t
mcos_list_instance_id(mcos_master_t * master, uint8_t idx)
{
	mcos_instance_t	  * instance = NULL;
	mInfo_t			  * mInfo;
	fcb_t			  * pFcb;
	int32_t				id, i;

	if ( idx > MAX_NUM_MCOS )
		return E_ERROR;

	printf("Master instance (%d) name: [%s]\n", idx, master->instance[idx].name);
	printf("Instance ID             ring Name\n");
	// Loop looking for valid instances with a valid ring.
	for(id = 0; id < MAX_NUM_MCOS; id++) {
		instance = &master->instance[id];
		if ( (instance->mInfo == NULL) || (idx != id) )
			continue;

		mInfo = instance->mInfo;

		for(i = 0; i < mInfo->num_fibers; i++) {
			pFcb = &mInfo->fcbs[i];
			if ( pFcb->state != FCB_FREE ) {
				printf(" %02x:%02x:%04x %16p %s\n",
						mcos_instance_index(instance->instance_id),
						mcos_instance_lcore(instance->instance_id),
						pFcb->pid, pFcb->msg_ring, pFcb->name);
			}
		}
	}

	return E_ERROR;
}

/**************************************************************************//**
*
* mcos_list_fibers - List all fibers of a MCOS instance
*
* DESCRIPTION
* List all fibers of an instance. This routine does not stop the MCOS instance
* and can return some odd state values.
*
* TODO: The only way is to send a high priority interrupt to the MCOS instance
* and have it display the fibers.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_list_fibers(mInfo_t * mInfo)
{
	int			i;

	printf("Pid  Name             Status       Priority Semaphore          ring:Name\n");
	for(i=0; i<mInfo->num_fibers; i++) {
		if ( mInfo->fcbs[i].state == FCB_FREE )
			continue;
		printf("%3d: %-16s %-12s %6d     %4d %16p:%-16s\n", i, mInfo->fcbs[i].name,
				mcos_state(mInfo->fcbs[i].state), mInfo->fcbs[i].priority,
				mInfo->fcbs[i].semid, mInfo->fcbs[i].msg_ring,
				(mInfo->fcbs[i].msg_ring)?mInfo->fcbs[i].msg_ring->name : "");
	}
}

static uint64_t
_max_stack( fcb_t * fcb )
{
	uint8_t	  * base = (uint8_t *)fcb->uctx->uc_stack.ss_sp;
	uint8_t	  * p = base;
	uint8_t	  * sp = (uint8_t *)fcb->uctx->uc_mcontext.gregs[REG_RSP];

	while ( (*p == 0x5a) && (p != sp) )
		p++;
	return (uint64_t)(fcb->uctx->uc_stack.ss_size - (p - base));
}

/**************************************************************************//**
*
* mcos_stack_check - List all fibers of a MCOS instance and display the stack information
*
* DESCRIPTION
* List all fibers of an instance with its stack usage. This routine does not
* stop the MCOS instance and can return some odd stack values.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_stack_check(mInfo_t * mInfo)
{
	int			i;
	fcb_t	  * fcb;
	uint64_t	u, sp, s, w, x;

	printf("Pid  Name             Status       Stack Size     Used   %%Used  Max Used\n");
	for(i=0; i<mInfo->num_fibers; i++) {
		if ( (fcb = &mInfo->fcbs[i])->state == FCB_FREE )
			continue;
		sp = fcb->uctx->uc_mcontext.gregs[REG_RSP];
		s = fcb->uctx->uc_stack.ss_size;
		u = (s - (sp - (uint64_t)fcb->uctx->uc_stack.ss_sp));
		w = (u * 100) / s;
		x = (u * 100) - s;
		printf("%3d: %-16s %-12s %10ld %8ld %3ld.%2ld%% %8ld\n", i, fcb->name,
				mcos_state(fcb->state), fcb->uctx->uc_stack.ss_size,
				u, w, (w) ? (x * 100)/s : (u * 10000)/s,
				_max_stack(fcb));
	}
}
