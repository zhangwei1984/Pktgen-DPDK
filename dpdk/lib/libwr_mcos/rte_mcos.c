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
#include <strings.h>
#include <string.h>
#include <unistd.h>
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
#include <rte_timer.h>

#include "rte_mcos.h"

#ifdef RTE_ENABLE_VIPER_API
__thread viper_t	  * __viper;
#endif

/**************************************************************************//**
*
* mcos_resched - Reschedule the current fiber
*
* DESCRIPTION
* Reschedule the current fiber for the next highest fiber priority.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_resched(mInfo_t * mInfo)
{
	fcb_t	  * cfcb;
	fcb_t	  * nfcb;
	int16_t		pid;
	uint32_t	bitmap;

	// Grab and save the vector bitmap and clear the bitmap.
	bitmap = mcos_clear_interrupts(mInfo);

	// Look for interrupts and add the fcb_t to the ready queue.
	if ( unlikely(bitmap) ) {
		int32_t		bit;

		dbgPrintf("Bitmap Vectors %08x\n", bitmap);

		// Loop testing the bitmap bits to see which vectors are active.
		while( (bit = ffs(bitmap)) ) {
			bit--;
			// If the vector array value is not EMPTY then make the pid ready
			if ( mInfo->vector[bit] != FCB_EMPTY ) {
				// Can not call mcos_resume inside of mcos_resched its recursive.
				mcos_ready(mInfo, mInfo->vector[bit], RESCHED_NO);
			}
			// Clear the bit and loop again
			bitmap &= ~(1 << bit);
		}
	}

	// If the current thread and we are the highest priority then continue running.
	if ( ((cfcb = mInfo->curr)->state == FCB_CURR) &&
		 (mcos_lastkey(mInfo, mInfo->rdytail) < cfcb->priority) ) {
		return;
	}

	// Force context switch
	if ( unlikely(cfcb->state == FCB_CURR) ) {
		cfcb->state = FCB_READY;
		cfcb->semid	= 0;
		mcos_insert(mInfo, cfcb->pid, mInfo->rdyhead, cfcb->priority);
	}

	// MCOS exits when it does not have any fibers to run.
	if ( unlikely((pid = mcos_getlast(mInfo, mInfo->rdytail)) == E_EMPTY) ) {
		dbgPrintf("No more fibers to run!\n");
		return;
	}

	nfcb = mcos_getfcb(mInfo, pid);

	// Setup for the new state.
	nfcb->state		= FCB_CURR;
	mInfo->curr		= nfcb;
	mInfo->currpid	= pid;

	// Count the number of times the scheduler has been called.
	mInfo->scheduler_cnt++;

	dbgPrintf("Switch to (%s) state: %s\n", nfcb->name, mcos_state(nfcb->state));
	swapcontext(cfcb->uctx, nfcb->uctx);
}

/**************************************************************************//**
*
* done_routine - The routine called when a task exits.
*
* DESCRIPTION
* The last function called when a task exits.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
done_routine(void * arg1, __attribute__ ((unused)) mcos_config_t * cfg)
{
	mInfo_t	  * mInfo = (mInfo_t *)arg1;

	dbgPrintf("Fiber (%s) is done for master %p\n", mInfo->curr->name, cfg->master);
	mcos_kill( mInfo, mcos_getpid(mInfo, NULL) );
}

/**************************************************************************//**
*
* __spawn - Internal routine to help spawn a new fiber
*
* DESCRIPTION
* The internal routine helps return a fcb_t * to a create fiber.
*
* RETURNS: NULL if error or pointer to fcb_t
*
* SEE ALSO:
*/

int32_t
__spawn(mInfo_t * mInfo, c8_t * name, int16_t prio, char * stack,
						int32_t stacksize, func_t func, void * arg, fcb_t ** ppFcb)
{
	int16_t		pid;

	if ( (stacksize == 0) || (stacksize < DEFAULT_STACK_SIZE) )
		stacksize = DEFAULT_STACK_SIZE;

	if ( (mInfo == (mInfo_t *)0) || (pid = __newpid(mInfo)) == 0 ) {
		dbgPrintf("mInfo %p, pid %d\n", mInfo, pid);
		return E_ERROR;
	}
	dbgPrintf("New pid %d for %s\n", pid, name);

	(*ppFcb) = &mInfo->fcbs[pid];
	if ( ((*ppFcb)->state != FCB_FREE) || ((*ppFcb)->pid != pid) ) {
		dbgPrintf("State is %s FCB_FREE or pid %d does not match %d\n",
				((*ppFcb)->state != FCB_FREE) ? " not" : "", pid, (*ppFcb)->pid);
		(*ppFcb)->state		= FCB_FREE;
		return E_ERROR;
	}

	// Round the size to the multiple of (1 << STACK_SHIFT)
	stacksize += ((1 << STACK_SHIFT) - 1);
	stacksize >>= STACK_SHIFT;		// Make the size a multiple by shifting off the lower bits

	dbgPrintf("Name (%s) stacksize %d(%d)\n", name, stacksize, (stacksize << STACK_SHIFT));

	(*ppFcb)->pid		= pid;
	(*ppFcb)->priority	= prio;
	(*ppFcb)->state		= FCB_SUSPEND;
	(*ppFcb)->stacksize	= stacksize;			// Size is in (STACK_SHIFT) sizes
	strncpy( (char *)(*ppFcb)->name, name, sizeof((*ppFcb)->name) );

	// Allocate the ucontext_t data structure, align to 8 byte boundary
	(*ppFcb)->uctx = (ucontext_t *)rte_zmalloc("uctx", sizeof(ucontext_t), 0);
	if  ( (*ppFcb)->uctx == NULL )
		rte_panic("Unable to allocate uctx\n");

	// Allocate the stack size
	if ( stack == NULL ) {
		(*ppFcb)->stack = rte_zmalloc("stack", stacksize << STACK_SHIFT, 0);
		if ( (*ppFcb)->stack == NULL )
			rte_panic("unable to allocated stack\n");
	} else
		(*ppFcb)->stack = stack;
	memset((*ppFcb)->stack, 0x5a, (stacksize << STACK_SHIFT));

	dbgPrintf("pFcb %p (%s)\n", (*ppFcb), (*ppFcb)->name);

	// Setup the stack location and size, then link to a fall back context.
	getcontext((*ppFcb)->uctx);
	(*ppFcb)->uctx->uc_stack.ss_sp	= (*ppFcb)->stack;
	(*ppFcb)->uctx->uc_stack.ss_size= (*ppFcb)->stacksize << STACK_SHIFT;
	(*ppFcb)->uctx->uc_link			= mInfo->done;
	makecontext((*ppFcb)->uctx, (void *)func, 2, mInfo, arg);

	return E_OK;
}

/**************************************************************************//**
*
* mcos_spawn - External mcos spawn routine.
*
* DESCRIPTION
* Spawn a new fiber with the given arguments.
*
* RETURNS: return the pid value or E_ERROR on error
*
* SEE ALSO:
*/

int16_t
mcos_spawn( mInfo_t * mInfo, c8_t * name, int16_t priority,
		uint32_t stacksize, func_t start_routine, void * arg )
{
	fcb_t	  * pFcb;

	dbgPrintf("Entry (%s)\n", name);

	if ( __spawn(mInfo, name, priority, NULL, stacksize, start_routine, arg, &pFcb) == E_ERROR )
		return E_ERROR;

	dbgPrintf("Leave (%s)\n", name);

	return pFcb->pid;
}

/**************************************************************************//**
*
* mcos_interrupt - Set the interrupt or signal in the bitmap
*
* DESCRIPTION
* Exit the current fiber with an error value.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_interrupt(mInfo_t * mInfo, int32_t v, int32_t arg)
{
	int16_t		pid;

	// Special handling of the clock interrupt.
	if ( likely(v == CLOCK_VECTOR) ) {

		mInfo->clk++;

		// Is the clock been deferred by the user, if so bump clkdiff and return
		if ( unlikely(mInfo->defclk) ) {
			dbgPrintf("defclk is set %d\n", mInfo->clkdiff);
			mInfo->clkdiff++;
			return;
		}

		// Sleep queue Not Empty flag.
		if ( likely(mInfo->slnempty) ) {

			mcos_set_bit(&mInfo->vector_bitmap, v);

			// Only decrease the value if it is not zero.
			if ( mInfo->sltop->key && --mInfo->sltop->key )
				return;
		}
	}

	// When the vector is not empty we can set the arg and enable the bit
	// in the vector bitmap for mcos_resched to active the interrupt fiber.
	if ( unlikely((pid = mInfo->vector[v]) != FCB_EMPTY) ) {
		mcos_getfcb(mInfo, pid)->vector_arg	= arg;
		mcos_set_bit(&mInfo->vector_bitmap, v);
	}
}

/**************************************************************************//**
*
* mcos_clock_intr - High level Interrupt handler for the clock.
*
* DESCRIPTION
* Handle all interrupts for clock ticks.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
mcos_clock_intr(mInfo_t * mInfo, __attribute__ ((unused)) void * arg2)
{
	for(;;) {
		// Wake up everyone waiting on the clock interrupt.
		mcos_wakeup(mInfo, RESCHED_YES);

		// Go back to sleep until the next clock tick.
		mcos_suspend(mInfo, mInfo->currpid);
	}
}

/**************************************************************************//**
*
* mcos_launch - Helper routine to launch MCOS on an lcore
*
* DESCRIPTION
* Helper routine to launch MCOS on to a lcore using DPDK APIs.
*
* RETURNS: E_ERROR or E_OK
*
* SEE ALSO:
*/

static int32_t
mcos_launch(void * arg)
{
	mcos_config_t * config = (mcos_config_t *)arg;
	mcos_instance_t * instance;

	instance = (mcos_instance_t *)mcos_alloc_instance((mcos_master_t *)config->master);
	if ( unlikely(instance == NULL) )
		return E_ERROR;

	// Save a copy of the instance pointer
	config->instance	= instance;

	// Copy config to private copy.
	*(mcos_config_t *)&instance->config = *config;
	rte_wmb();

	// Start up a version of MCOS for this lcore.
    return mcos_initialize(&instance->mInfo, &instance->config);
}

/**************************************************************************//**
*
* mcos_remote_launch - Create and launch a MCOS system on a given lcore
*
* DESCRIPTION
* Launch the first MCOS thread and use the mcos_launch running on the lcore
* to initialize the system. The mcos_launch() will also call the callback
* routine in the application to finish initializing the application threads.
*
* RETURNS: E_ERROR or E_OK
*
* SEE ALSO:
*/

int32_t
mcos_remote_launch(__attribute__ ((unused)) mcos_master_t * master,
							mcos_config_t * config, uint32_t lcore)
{
	if ( unlikely(lcore > MAX_NUM_MCOS) )
		return E_ERROR;

	config->lcore = lcore;
	// Start running the MCOS instance on the lcore.
	return rte_eal_remote_launch(mcos_launch, config, lcore);
}

static void
__process_tick(__attribute__((unused)) struct rte_timer *tim, void * arg)
{
	mInfo_t	  * mInfo = (mInfo_t *)arg;
	fcb_t	  * pFcb;
	int			i;

	for(i=0; i<mInfo->num_fibers; i++) {
		pFcb = &mInfo->fcbs[i];
		if ( pFcb->timer ) {
			pFcb->timer->timeout--;
		}
	}

	mcos_interrupt(mInfo, CLOCK_VECTOR, 0);
}

/**************************************************************************//**
*
* __timerInit - Initialize the internal timer system.
*
* DESCRIPTION
* Setup the Timer system to be used for the timer routines, using the DPDK
* timer functions.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
__timerInit(mInfo_t * mInfo)
{
    int lcore_id = rte_lcore_id();

    /* init RTE timer library */
    rte_timer_subsystem_init();

    /* init timer structures */
    rte_timer_init(&mInfo->timer0);

    /* load timer0, every Nth of a second, on timer lcore, reloaded automatically */
    rte_timer_reset(&mInfo->timer0, mInfo->hz/mcos_get_tickrate(), PERIODICAL,
    		lcore_id, __process_tick, mInfo);
}

/**************************************************************************//**
*
* __clkinit - Initialize the clock routine.
*
* DESCRIPTION
* Initialize the clock structure and get ready to be started.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
__clkinit(mInfo_t * mInfo)
{
	mInfo->tickrate	= DEFAULT_MCOS_TICK_RATE;
	mInfo->slnempty = 0;
	mInfo->clkdiff	= 0;
	mInfo->defclk	= 0;
	mInfo->sleepq	= __newq(mInfo);
	mInfo->flags	|= CLOCK_FLAG;

	mcos_clkstop(mInfo);
}

/**************************************************************************//**
*
* __clkstart - Start up the clock
*
* DESCRIPTION
* Start up the clock, which means mcos_clkinit() should have been called
* first.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
__clkstart(mInfo_t * mInfo)
{
	int32_t	makeup;
	int16_t	next;
	qentry_t  * q = &mInfo->q[0];

	if ( (mInfo->defclk <= 0) || (--mInfo->defclk > 0) )
		return;

	makeup			= mInfo->clkdiff;
	mInfo->clkdiff	= 0;

	// Sleep queue is not empty flag.
	if ( mInfo->slnempty ) {
		// Decrease the tick count in the sleepq list.
		for( next = mcos_firstid(mInfo, mInfo->sleepq);
				next < mInfo->num_fibers && q[next].key < makeup;
				next = q[next].next) {
			makeup -= q[next].key;
			q[next].key = 0;
		}
		// Do we have more make up ticks then we have entries.
		if ( next < mInfo->num_fibers )
			q[next].key -= makeup;

		// Wake up everyone on the sleepq and let them fight who runs first.
		mcos_wakeup(mInfo, RESCHED_YES);
	}
}

/**************************************************************************//**
*
* nullproc - lowest priority task in the system
*
* DESCRIPTION
* The first and lowest priority task in the system
*
* RETURNS: N/A
*
* SEE ALSO:
*/

static void
nullproc(mInfo_t * mInfo, mcos_config_t * cfg)
{
	int16_t		pid;
	char	name[MAX_NAME_LENGTH+1];

	// initialize the clock routine.
	__clkinit( mInfo );

	// Create a high priority thread to handling clock interrupts.
	pid = mcos_spawn(mInfo, "ClockIntr", MAX_PRIO, 0, mcos_clock_intr, cfg->arg);
	mcos_set_vector(mInfo, CLOCK_VECTOR, pid);

	// Start the clock running.
    __clkstart( mInfo );

    // initialize the MCOS instance information in the master list.
    snprintf(name, sizeof(name), "MCOS_%d:%d",
    		mInfo->instance->instance_id.s.idx,
    		mInfo->instance->instance_id.s.lcore);
    mcos_initialize_instance(mInfo, name, RTE_INSTANCE_MSG_COUNT);

    dbgPrintf("Call the callback routine\n");

	// Do the user callback only once and pass in user config pointer.
    mcos_init_callback(mInfo, cfg);

    dbgPrintf("Endless loop for nullproc\n");

	// when or if we have interrupts then we must spin calling mcos_resched().
    do {
    	// Count the number of loops in nullproc fiber
    	mInfo->nullproc_loops++;

    	// I would like to call rte_eal_mwait() at this point, but we can not
    	// in user space.
		mcos_resched(mInfo);

	} while( likely((mInfo->flags & STOP_FLAG) == 0) );
}

/**************************************************************************//**
*
* nullproc_init - Start up the first fiber
*
* DESCRIPTION
* Setup the first process or fiber.
*
* RETURNS: Return the NULLPROC_PID
*
* SEE ALSO:
*/

static uint16_t
nullproc_init(mInfo_t * mInfo, mcos_config_t * cfg)
{
	int16_t		pid = NULLPROC_PID;
	fcb_t	  * pFcb;

	pFcb = &mInfo->fcbs[pid];

	// Round the size to the multiple of (1 << STACK_SHIFT)
	pFcb->stacksize		= (DEFAULT_STACK_SIZE >> STACK_SHIFT);

	dbgPrintf("Name (NullProc) stacksize %d(%d)\n",
			pFcb->stacksize, (pFcb->stacksize << STACK_SHIFT));

	pFcb->pid			= pid;
	pFcb->priority		= NULLPROC_PRIO;
	pFcb->state			= FCB_CURR;
	strncpy( (char *)pFcb->name, "NullProc", sizeof(pFcb->name)-1 );

	// Allocate the ucontext_t data structure, align to 8 byte boundary
	pFcb->uctx = (ucontext_t *)rte_zmalloc("uctx", sizeof(ucontext_t), 0);
	if ( pFcb->uctx == NULL )
		rte_panic("unable to allocate uctx\n");

	// Allocate the stack size
	pFcb->stack		= rte_zmalloc("stack", pFcb->stacksize << STACK_SHIFT, 0);
	if ( pFcb->stack == NULL )
		rte_panic("Unable to allocate stack\n");
	memset(pFcb->stack, 0x5a, (pFcb->stacksize << STACK_SHIFT));

	mInfo->curr		= pFcb;
	mInfo->currpid	= pid;

	// Grab a context for the fiber
	getcontext(pFcb->uctx);

	// Setup the stack location and size, then link to a fall back context.
	pFcb->uctx->uc_stack.ss_sp		= pFcb->stack;
	pFcb->uctx->uc_stack.ss_size	= pFcb->stacksize << STACK_SHIFT;
	pFcb->uctx->uc_link				= mInfo->done;
	makecontext(pFcb->uctx, (void *)nullproc, 2, mInfo, cfg);

	dbgPrintf("pFcb %p (%s)\n", pFcb, pFcb->name);

	return pid;
}

/**************************************************************************//**
*
* mcos_initialize - initialize the mcos system and define number fibers, semaphores, ..
*
* DESCRIPTION
* Initialize the MCOS system and start the scheduler running.
*
* RETURNS: E_ERROR or E_OK.
*
* SEE ALSO:
*/

int32_t
mcos_initialize( mInfo_t ** ret, mcos_config_t * cfg)
{
	mInfo_t		  * mInfo = NULL;
	uint32_t		i, num_queues;
	sentry_t	  * s;
	mentry_t	  * m;

	if ( cfg->num_fibers == 0 )
		cfg->num_fibers = DEFAULT_NUM_FIBERS;
	if ( cfg->num_semas == 0 )
		cfg->num_semas = DEFAULT_NUM_SEMAS;
	if ( cfg->num_mutexs == 0 )
		cfg->num_mutexs = DEFAULT_NUM_MUTEXS;

	num_queues = NUM_INTERNAL_QUEUES + (cfg->num_semas * 2) + (cfg->num_mutexs * 2) + cfg->num_fibers;

	if ( unlikely(num_queues >= (MAXSHORT - 1)) ) {
		printf("*** Number of (num_fibers(%d) + (semaphores(%d) * 2) + %d) exceed %d\n",
				cfg->num_fibers, cfg->num_semas, NUM_INTERNAL_QUEUES, (MAXSHORT - 1));
		goto exit_error;
	}

	// NOTE: Make sure sizeof(mInfo_t) is aligned and multiples of alignment
	mInfo = (mInfo_t *)rte_zmalloc("mInfo", sizeof(mInfo_t), 0);
	if ( mInfo == NULL )
		goto exit_error;

	dbgPrintf("Entry num_fibers %d, num_semas %d, num_queues %d num_mutexs %d master %p arg %p\n",
			cfg->num_fibers, cfg->num_semas, num_queues, cfg->num_mutexs, cfg->master, cfg->arg);

	if ( likely(ret != NULL) )
		*ret = mInfo;

    mInfo->hz			= rte_get_hpet_hz();

	// Allow the mInfo to point back to the instance and master structures
	mInfo->master		= cfg->master;
	mInfo->instance		= cfg->instance;
	mInfo->lcore		= cfg->lcore;

	// Save the initial values
	mInfo->num_fibers	= cfg->num_fibers;
	mInfo->num_queues	= num_queues;
	mInfo->num_semas	= cfg->num_semas;
	mInfo->num_mutexs	= cfg->num_mutexs;

	// Save the information
	mInfo->nextfcb		= cfg->num_fibers - 1;
	mInfo->nextsema		= cfg->num_semas - 1;
	mInfo->nextqueue	= cfg->num_fibers;
	mInfo->nextmutex	= cfg->num_mutexs - 1;

	mInfo->vector		= (int16_t *)rte_zmalloc("vector", MAX_VECTORS * sizeof(int16_t), 0);
	if ( mInfo->vector == (int16_t *)0 )
		goto exit_error;

	for(i=0; i<MAX_VECTORS; i++)
		mInfo->vector[i] = FCB_EMPTY;

	// Allocate the FCB structures
	mInfo->fcbs = (fcb_t *)rte_zmalloc("fibers", cfg->num_fibers * sizeof(fcb_t), 0);
	if ( mInfo->fcbs == (fcb_t *)0 )
		goto exit_error;

	// Set the first pointer the first PID
	mInfo->currpid = (cfg->num_fibers - 1);
	mInfo->curr = &mInfo->fcbs[mInfo->currpid];

	// Allocate the Queue structures
	mInfo->q = (qentry_t *)rte_zmalloc("queues", num_queues * sizeof(qentry_t), 0);
	if ( mInfo->q == (qentry_t *)0 )
		goto exit_error;

	// Allocate the Semaphore structures
	mInfo->sem = (sentry_t *)rte_zmalloc("semas", cfg->num_semas * sizeof(sentry_t), 0);
	if ( mInfo->sem == (sentry_t *)0 )
		goto exit_error;

	// Allocate the Semaphore structures
	mInfo->mutex = (mentry_t *)rte_zmalloc("mutex", cfg->num_mutexs * sizeof(mentry_t), 0);
	if ( mInfo->mutex == (mentry_t *)0 )
		goto exit_error;

	// Setup the FCB structures
	for(i = 0; i < cfg->num_fibers; i++) {
		mInfo->fcbs[i].pid		= i;
		mInfo->fcbs[i].state	= FCB_FREE;
	}

	// Initialize semaphores
	for (i=0 ; i < cfg->num_semas ; i++) {
		(s = &mInfo->sem[i])->state = S_FREE;
		s->tail = 1 + (s->head = __newq(mInfo));
	}

	// Initialize mutex semaphores
	for (i=0 ; i < cfg->num_mutexs ; i++) {
		(m = &mInfo->mutex[i])->owner = M_FREE;
		m->tail = 1 + (m->head = __newq(mInfo));
	}

	// initialize ready queue list
	mInfo->rdytail = (mInfo->rdyhead = __newq(mInfo)) + 1;

	if ( (mInfo->rdyhead == E_ERROR) || (mInfo->rdytail == E_ERROR) )
		goto exit_error;

	mInfo->done = (ucontext_t *)rte_zmalloc("done", sizeof(ucontext_t), 0);
	if ( mInfo->done == NULL )
		rte_panic("Unable to allocate done context\n");;

	// Initialize the timer structures
	__timerInit(mInfo);

	// setup the done routine context.
	getcontext(mInfo->done);
	mInfo->done->uc_stack.ss_sp		= rte_zmalloc("ss_sp", DEFAULT_STACK_SIZE, 0);
	if ( mInfo->done->uc_stack.ss_sp == NULL )
		rte_panic("Unable to allocate stack space\n");

	mInfo->done->uc_stack.ss_size	= DEFAULT_STACK_SIZE;
	mInfo->done->uc_link			= NULL;
	makecontext(mInfo->done, (void *)done_routine, 2, mInfo, cfg);

	// Set the flag that we have been initialized.
	mInfo->flags |= INIT_FLAG;

	nullproc_init( mInfo, cfg );

	dbgPrintf("Set context to the nullproc routine and jump into nullproc.\n");

	// Leave the mcos_init routine via setcontext call.
	setcontext(mcos_getfcb(mInfo, NULLPROC_PID)->uctx);

	return E_OK;

exit_error:
	mcos_cleanup(mInfo);
	return E_ERROR;
}

/**************************************************************************//**
*
* mcos_msg_dispatch - Dispatch system for messages from other instances.
*
* DESCRIPTION
* Process any messages and dispatch them to the correct MCOS instance and process.
* The mcos_dispatch() routine must be called often to process an instance set of
* messages. The more often this routine is called the faster the messages are
* delivered to the MCOS fibers ring.
*
* We attempt to only process message from a single instance up to the watermark
* value, then leave to be called back to process another instance. In the routine
* we skip any instances that are not configured to make each call productive as
* possible.
*
* RETURNS: E_ERROR or Number of messages
*
* SEE ALSO:
*/

void
mcos_msg_dispatch(mcos_master_t * master)
{
	int32_t				j, rx_cnt;
	mcos_addr_t			dst_id;
	mcos_data_t		  * data;
	mInfo_t			  * mInfo;
	mcos_instance_t	  * instance;

	mInfo = master->instance[master->curr_index++].mInfo;
	if ( master->curr_index > master->instance_count )
		master->curr_index = 0 ;

	if ( mInfo ) {
		instance = mInfo->instance;

		// Grab a message from the instance ring.
		if ( (rx_cnt = mcos_recv_instance_burst(instance, master->msg_buff, master->msg_cnt)) > 0 ) {
			dbgAssert(rx_cnt <= master->msg_cnt);

			data = master->msg_buff;
			for(j = 0; j < rx_cnt; j++) {
				dst_id.data = data->dst_id.data;

				// Send the message to the correct instance/process ring.
				if ( mcos_send_message(instance->mInfo, dst_id, data) > 0 )
					master->dispatch_sent++;
				else
					master->dispatch_error++;
				data++;
			}
		}
	}
}
