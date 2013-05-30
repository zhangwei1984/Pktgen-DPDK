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

/* phil.c - Dining Philosophers problem */

/**
DESCRIPTION
This demo demonstrates multitasking within the VxWorks kernel by providing two 
solutions to Dijkstra's famous dining philosophers problem. The main goal 
of the problem is to find ways to avoid both deadlock and starvation 
when a finite set of actors share a finite set of resources that
can be used by one actor only at a time.

The problem is described as follows: five philosophers sit around a table to
think and eat. Between each adjacent philosopher there is a fork. When a
philosopher is done thinking she/he needs to grab the two forks on his/her
immediate right and left to be able to eat. When the philosopher is done eating
she/he puts down the forks and go back to thinking for a while till he or she
is hungry again, etc., etc.

In this implementation the number of philosophers can be changed (five being
the default). Also the duration of a philosopher's thinking phase is not the
same for all. This brings a more realistic touch to the situation since,
usually, actors accessing a resource do not all have the same frequency of
access to that resource. This implementation simply lets a philosopher think for
a number of seconds equal to the philosopher's order number around the table
(i.e. 1 to x) instead of some random time or an equal amount of time for all.

The two solutions implemented here are 1) a ticket-based one, and 2) a
claim-based one. The ticket-based solution is fair in the sense that all
philosophers get to access the resource for the same amount of time in average
whether they think quick (or shallowy...) or long. The drawback is that the
faster thinkers get to wait more for accessing the resource. The claim-based
solution is addressing this by letting the faster thinkers access the resource
as long as it is not claimed by another philosopher (in which case the
requestor still has to wait until the other philosopher has gotten a chance to
use the resource).

INCLUDE FILES: N/A
*/

/* Includes */
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
#include <rte_scrn.h>

#include "rte_mcos.h"
#include "mcos_main.h"
#include "phil.h"

int 		quit = 0;
info_t	  * infos[RTE_MAX_LCORE][MAX_PHILS];
saved_t		saved[RTE_MAX_LCORE][MAX_PHILS];
uint64_t	cnt_id = 0;

const char * doing_str[] = {
		"  thinking..........",	// 0
        "  wants to eat......",	// 1
        "  wait for forks....",	// 2
        "  eating............",	// 3
        "  ask right fork....",	// 4
        "  wait right fork...",	// 5
        "  ask left fork.....",	// 6
        "  wait left fork....",	// 7
        "  give all forks....",	// 8
        "  ------------------",	// 9
        NULL
};

const uint32_t	ids[5] = {
		0x00010000,
		0x01020000,
		0x02030000,
		0x03040000,
		0x04050000
};

/*******************************************************************************
*
* philDemo - Dining philosopher demo
* 
* This is the main entry of the dining philosopher's demo.
* 
* Options to this demo are:
*
* \is
* \i solutionNum
* Selection the solution to run the demo. 1 = ticket-based solution, 2=
* claim-based solution. Default (0) is set to solution #1.
* \i philosopherNum
* Indicate the number of philosopher tasks to run. Default (0) is 5 tasks.
* \i duration
* Length of the time to run the demo. Default (0) is run forever.
* \ie
*
* If the demo is left to run forever (the default) then it will stop and
* print statistics when the main task is resumed from the shell.
*
* RETURNS: N/A
* ERRNO: N/A
*/

void philDemo( mInfo_t * mInfo, __attribute__ ((unused))void * arg )
{
    int solutionNum = 0;     /* Solution to run (1 or 2). Default (0) is sol. 1 */
    int philosopherNum = 0;  /* Number of philosopher tasks. Default (0) is 5 */
    int duration = 0;        /* Duration of demo. Default (0) is forever */
    unsigned int        solution;       /* Either one of the impl. solutions */
    PHILOSOPHERS_ATTR   philosAttr;     /* Philosopher attributes */
    FORKS_ATTR          forksAttr;      /* Fork attributes */
    STATS               stats;          /* Statistic data */
    int                 i;    			/* Loop counter */
    int     demoDuration = 0;           /* By default the demo runs forever */
    char				name[16];
    info_t			  * info;
    BOOL    philosophersNotReady = TRUE;/* FALSE when all philosopher tasks */
                                        /* are ready to go on */

    /* By default let's run the claim-based solution */

    solution = TICKET_BASED;

    /* Default number of philosophers */

    philosAttr.numPhil = MAX_PHILS;

    /* Decipher the options, if any is passed */

    if ((solutionNum > 0) && (solutionNum < 3))
        solution = solutionNum;

    if (philosopherNum > 5)
        philosAttr.numPhil = philosopherNum;

    demoDuration = duration;

    /* There is as much forks as there are philosophers */

    forksAttr.numForks = philosAttr.numPhil;

    /* Create synchronization semaphore */

    philosAttr.syncSemId = mcos_screate(mInfo, 0);

    /* Create the various arrays used in this implementation */

    philosAttr.pPhil    = (int16_t *) calloc ((size_t)philosAttr.numPhil, sizeof(int16_t));
    philosAttr.iAmReady = (BOOL *) calloc ((size_t)philosAttr.numPhil, sizeof(BOOL));
    stats.foodIntake    = (unsigned int *) calloc ((size_t)philosAttr.numPhil, sizeof (unsigned int));
    forksAttr.forks     = (struct fork *)  calloc ((size_t)philosAttr.numPhil, sizeof (A_FORK));
    stats.goHungry      = (unsigned int *) calloc ((size_t)philosAttr.numPhil, sizeof (unsigned int));

    /* Initialize the owner of the forks */

    for (i = 0; i < philosAttr.numPhil; i++)
        forksAttr.forks[i].philosopher = NOBODY;

    /* Create mutex semaphore protecting the critical sections */

    forksAttr.forkMutex = mcos_mcreate(mInfo);

    /* Create philosopher tasks */

    strcpy(name, "tPhil0");
    for (i = 0; i < philosAttr.numPhil; i++)
        {
    	info = (info_t *)calloc(1, sizeof(info_t));
    	infos[rte_lcore_id()][i]	= info;
    	info->doing					= 0;
    	info->idx					= i;
    	info->solution				= solution;
    	info->philosAttr			= &philosAttr;
    	info->forksAttr				= &forksAttr;
    	info->stats					= &stats;
    	snprintf(name, sizeof(name), "tPhil%d",i);
    	philosAttr.pPhil[i] = mcos_spawn(mInfo, name, 30, 0, (func_t)philosopherRun, info);
    	mcos_resume( mInfo, philosAttr.pPhil[i] );
        }

    /*
     * Wait for all the philosopher tasks to be pending on the synchronization
     * semaphore.
     */

    while (philosophersNotReady)
        {
        philosophersNotReady = FALSE;

        for (i = 0; i < philosAttr.numPhil; i++)
            {
            if (philosAttr.iAmReady[i] == FALSE) {
                philosophersNotReady = TRUE;
                dbgPrintf("Not Ready %d\n", i);
            }
            }
        dbgPrintf("Sleep for Ready\n");
        mcos_delay(mInfo, 2);
        }
    dbgPrintf("All Ready!\n");

    /* Let the philosophers start their meal */

    mcos_semflush(mInfo, philosAttr.syncSemId);

    /* Let the demo run as much as one wants */

    if (demoDuration == 0) {
    	do {
    		mcos_delay_ms(mInfo, 250);
    	} while( quit == 0 );
    } else
        mcos_sleep(mInfo, demoDuration);

    /* Then kill the philosopher tasks, print statistics and exit */

    for (i = 0; i < philosAttr.numPhil; i++)
        mcos_kill(mInfo, philosAttr.pPhil[i]);

    /* Free resources */

    mcos_sdelete (mInfo, philosAttr.syncSemId);
    free (philosAttr.pPhil);
    free (philosAttr.iAmReady);
    free (stats.foodIntake);
    free (stats.goHungry);
    mcos_mdelete (mInfo, forksAttr.forkMutex);
    free (forksAttr.forks);

    dbgPrintf("Finish philDemo %d\n", rte_lcore_id());
}

/*******************************************************************************
*
* philosopherRun - A philosopher task's run routine
* 
* This routine is the starting point for each of the philosopher task.
*
* RETURNS: N/A
* ERRNO: N/A
*
* \NOMANUAL
*/

void philosopherRun( mInfo_t * mInfo, info_t * info )
    {
	saved_t		* save = &saved[rte_lcore_id()][info->idx];
    int                 myNum		= info->idx;      /* This philosopher's order number */
    int                 solution	= info->solution;   /* Solution to execute */
    PHILOSOPHERS_ATTR * pPhilsAttr	= info->philosAttr; /* Philosophers attributes */
    FORKS_ATTR *        pForksAttr	= info->forksAttr; /* Forks attributes*/
    STATS *             pStats		= info->stats;      /* statistics data */
    pPhilsAttr->iAmReady[myNum] = TRUE;

    /* Philosopher waits till every one is ready */

    dbgPrintf("syncSemId %d (%s)\n", pPhilsAttr->syncSemId, mInfo->curr->name);

    mcos_wait(mInfo, pPhilsAttr->syncSemId);

    /* Now start thinking and eating */

    while (TRUE)
    	{
		info->doing		= 0;

    	if ( quit )
    		break;

    	dbgPrintf("Sleep\n");
    	save->count = 0;
        mcos_delay_ms(mInfo, (mcos_rand32(mInfo) & 0x3FF));

        info->doing		= 1;
        
        /*
         * If the forks cannot be acquired this implementation simply does
         * an active wait and checks again for the fork's availability after a
         * second of delay.
         */

        while (forksGet (mInfo, myNum, solution, pForksAttr, info) == FALSE)
            {
            if (solution == TICKET_BASED)
                {
            	info->doing		= 3;
                }
            if ( quit )
            	break;
            pStats->goHungry[myNum]++;
            dbgPrintf("Go Hungry %d\n", pStats->goHungry[myNum]);
        	save->count = 1;
            mcos_delay_ms(mInfo, (mcos_rand32(mInfo) & 0x3FF));
            }

        info->doing		= 4;

        pStats->foodIntake[myNum]++;
        dbgPrintf("Food Intake %d\n", pStats->foodIntake[myNum]);
    	save->count = 2;
        mcos_delay_ms(mInfo, mcos_rand32(mInfo) & 0x3FF);

        forksPut (mInfo, myNum, solution, pForksAttr, info);
        }
    free(info);
    }

/*******************************************************************************
* 
* forksGet - Get the right-hand and left-hand pair of forks
* 
* This routine attempts to acquire the forks a philosopher needs to be able to
* eat. Forks are always acquired in pair, i.e. if only one of the forks is
* available then no fork is acquired.
* 
* RETURNS: TRUE if both forks are acquired, FALSE otherwise.
* ERRNO: N/A
*
* \NOMANUAL
*/

BOOL forksGet
    (
    mInfo_t		  * mInfo,
    int             myNum,      /* Philosopher's order number */
    int             solution,   /* Solution to execute */
    FORKS_ATTR *    pForksAttr, /* Forks attributes*/
    info_t		  * info
    )
    {
    BOOL forksAcquired  = FALSE;
    BOOL passTurn       = FALSE;
    int  rightFork      = myNum;
    int  leftFork       = (myNum + 1) % pForksAttr->numForks;
	saved_t			  * save = &saved[rte_lcore_id()][info->idx];

    /* Acquire the mutex protecting the critical section */

	save->count = 3;
    mcos_take (mInfo, pForksAttr->forkMutex);

    if (solution == TICKET_BASED)
        {
        /* Check whether it is this philosopher's turn to get the forks */

        if (((pForksAttr->forks[rightFork].philosopher == myNum) ||
             (pForksAttr->forks[rightFork].philosopher == NOBODY)) &&
            ((pForksAttr->forks[leftFork].philosopher  == myNum) ||
             (pForksAttr->forks[leftFork].philosopher  == NOBODY)))
            forksAcquired = TRUE;
        }
    else /* Claim-based solution */
        {
        /* 
         * A fork can be acquired only if it is not in use and if no one else
         * has claimed it. If the fork is in use already then the philosopher
         * simply claims it and waits for it to be available. If the fork is
         * available but has already been claimed then the philosopher must
         * wait for his/her turn.
         * 
         * Note: if we were to randomize the think time of the philosophers it
         *       might be more efficient for a philosopher to claim the fork
         *       when the fork is available but has been claimed by the other
         *       philosopher (i.e. implement a two-slot queue for the fork).
         */

        if (pForksAttr->forks[rightFork].status == FORK_IN_USE)
            {
            pForksAttr->forks[rightFork].philosopher = myNum;
            passTurn = TRUE;
            info->doing		= 4;
            }
        else
            if ((pForksAttr->forks[rightFork].philosopher != myNum) &&
                (pForksAttr->forks[rightFork].philosopher != NOBODY))
                {
                passTurn = TRUE;

                info->doing	= 5;
                }

        if (pForksAttr->forks[leftFork].status == FORK_IN_USE)
            {
            pForksAttr->forks[leftFork].philosopher = myNum;
            passTurn = TRUE;

            info->doing		= 6;
            }
        else
            if ((pForksAttr->forks[leftFork].philosopher != myNum) &
                (pForksAttr->forks[leftFork].philosopher != NOBODY))
                {
                passTurn = TRUE;

                info->doing  = 7;
                }

        /* 
         * if both forks are available and claimed by no one else then the
         * philosopher may acquire them.
         */

        if (!passTurn)
            {
            pForksAttr->forks[rightFork].status      = FORK_IN_USE;
            pForksAttr->forks[leftFork].status       = FORK_IN_USE;
            pForksAttr->forks[rightFork].philosopher = NOBODY;
            pForksAttr->forks[leftFork].philosopher  = NOBODY;

            forksAcquired = TRUE;
            }
        }

    /* Release the mutex protecting the critical section */

    save->count = 4;
    mcos_give(mInfo, pForksAttr->forkMutex);

    return forksAcquired;
    }

/*******************************************************************************
* 
* forksPut - Put down the right-hand and left-hand pair of forks
* 
* This routine simply releases the forks a philosopher was using to eat.
* 
* RETURNS: N/A
* ERRNO: N/A
*
* \NOMANUAL
*/

void forksPut
    (
    mInfo_t		  * mInfo,
    int             myNum,      /* Philosopher's order number */
    int             solution,   /* Solution to execute */
    FORKS_ATTR *    pForksAttr, /* Forks attributes*/
    info_t		  * info
    )
{
    int rightFork = myNum;
    int leftFork  = (myNum + 1) % pForksAttr->numForks;
	saved_t			  * save = &saved[rte_lcore_id()][info->idx];

    /* Acquire the mutex protecting the critical section */
	save->count = 5;
    mcos_take (mInfo, pForksAttr->forkMutex);

    if (solution == TICKET_BASED)
        {
        /*
         * The forks are handed over to the philosophers on the immediate right
         * and immediate left of the philosopher that is done eating.
         */

        pForksAttr->forks[rightFork].philosopher = (rightFork ? rightFork - 1 :
                                                    (pForksAttr->numForks - 1));
        pForksAttr->forks[leftFork].philosopher = leftFork;

        info->doing  = 8;
        }
        else	/* Claim-based solution */
            {
            /* The forks are simply flagged as available */

            pForksAttr->forks[rightFork].status = FORK_AVAILABLE;
            pForksAttr->forks[leftFork].status  = FORK_AVAILABLE;
            }

    /* Release the mutex protecting the critical section */
    save->count = 6;
    mcos_give(mInfo, pForksAttr->forkMutex);
}

void phil_quit( void )
{
	quit = 1;
}

void phil_demo_start(mInfo_t * mInfo, void * arg)
{
	char		buf[32];

	// Setup the random number seed.
	mcos_srand32(mInfo, 0x19560630+(rte_lcore_id()*333));

    snprintf(buf, sizeof(buf), "PhilDemo%d", rte_lcore_id());
    mcos_resume(mInfo, mcos_spawn(mInfo, buf, 40, 0, (func_t)philDemo, arg));
}
