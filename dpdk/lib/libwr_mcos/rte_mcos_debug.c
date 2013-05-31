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

/*
DESCRIPTION
The internal debug message to format the debug message into the correct for
the dbgPrintf() macro. 

INCLUDE FILES: mcos_debug.h

\NOROUTINES
*/

/* includes */

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

#include "rte_mcos.h"

static  int8_t        dbgBuffer[DBG_BUF_SIZE];
        int8_t      * dbgBuf = dbgBuffer;
        FILE	    * dbgFile;

/**************************************************************************//**
* dbgInit - Initialize the debug routine.
* 
* DESCRIPTION
* Setup the debug system and set memory to the correct initial value.
* 
* RETURNS: E_OK
* 
* ERRNO: N/A
*/
status_t
mcos_dbgInit( int32_t log )
{
	dbgFile = ( log ) ? fopen("mcos_debug.log", "w") : stdout;

	return E_OK;
}

/**************************************************************************//**
*
* mcos_dbgListq - Dump out a list or queue
*
* DESCRIPTION
* Dump out a list or queue of information.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_dbgListq(mInfo_t * mInfo, c8_t * msg, int16_t head)
{
	qentry_t  * q = mInfo->q;
	int16_t		tail = head + 1;
	int			i, k;

	fprintf(dbgFile, "\n*** %s Queue: %d\n", msg, head);

	fprintf(dbgFile, "      Pid:   Next  Prev    Key Name               State\n");
	fprintf(dbgFile, "head %5d: %5d %5d %6d\n", head, q[head].next, q[head].prev, q[head].key);
	for(i = q[head].next, k=0; (i != tail) && (k < mInfo->num_queues); i = q[i].next, k++) {
		fprintf(dbgFile, "     %5d: %5d %5d %6d (%-16s) %s\n", i, q[i].next, q[i].prev,
				q[i].key, mInfo->fcbs[i].name, mcos_state(mInfo->fcbs[i].state));
	}
	fprintf(dbgFile, "tail %5d: %5d %5d %6d\n", tail, q[tail].next, q[tail].prev, q[tail].key);
	tail = mInfo->currpid;
	fprintf(dbgFile, "curr %5d: %5d %5d %6d (%-16s) %s\n\n", tail, q[tail].next, q[tail].prev,
			q[tail].key, mInfo->fcbs[tail].name, mcos_state(mInfo->fcbs[tail].state) );
}

/**************************************************************************//**
*
* mcos_dbgLists - Dump out a list on a semaphore
*
* DESCRIPTION
* Dump out a list or queue of information.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
mcos_dbgLists(mInfo_t * mInfo, c8_t * msg, int16_t sem)
{
	sentry_t  * s = mInfo->sem;
	qentry_t  * q = mInfo->q;
	int16_t		head = s[sem].head;
	int16_t		tail = head + 1;
	int			i, k;

	fprintf(dbgFile, "\n*** %s Semaphore: %d\n", msg, sem);

	fprintf(dbgFile, "      Pid:   Next  Prev    Key Name               State\n");
	fprintf(dbgFile, "sema %5d: %5d %5d %6d\n", head, q[head].next, q[head].prev, q[head].key);
	for(i = q[head].next, k=0; (i != tail) && (k < mInfo->num_fibers); i = q[i].next, k++) {
		fprintf(dbgFile, "     %5d: %5d %5d %6d (%-16s) %s\n", i, q[i].next, q[i].prev,
				q[i].key, mInfo->fcbs[i].name, mcos_state(mInfo->fcbs[i].state));
	}
	fprintf(dbgFile, "tail %5d: %5d %5d %6d\n", tail, q[tail].next, q[tail].prev, q[tail].key);
}

/**************************************************************************//**
* dbgPrintInfo - Print Debug information code routine.
*
* DESCRIPTION
* A routine called from dbgPrintf() macro to print a debug message on the
*
* console port. The output displays the message plus pre-pending task name,
* file name, function name and line number to the message.
* 
* The routine is called with a set of values and a message string.
* 
* \is
* \i <pFunc> String pointer of the function from __FUNCTION__ macro.
* \i <pFile> String pointer of the filename __FILE__ macro.
* \i <line> A 32 bit integer of the line number __LINE__ the message statment
* appeared.
* \ie
* 
* RETURNS: N/A.
* 
* ERRNO: N/A
*/
void
dbgPrintInfo( mInfo_t * mInfo, c8_t * pFunc, c8_t * pFile,
                        const int32_t line, int8_t * pBuf )
{
    int8_t    data[MAX_DBG_MESSAGE + 1];
    c8_t	* p;
    int32_t   len;
    
    p = (c8_t *)strrchr(pFile, '\\');
    if ( p == NULL )
        p = (c8_t *)strrchr(pFile, '/');

    /*
     * Make sure the message fits in the buffer space and format the first
     * part of the message.
     */
    snprintf((char *)data, sizeof(data), "%d:%-10s (%s:%d)", rte_lcore_id(), (mInfo->curr) ? mInfo->curr->name : "???",
    		(!p) ? pFile : ++p, line);

    /* 
     * Verify the length of the message is 45 or greater and if not then
     * pad the string to make it 45 bytes of information before the message.
     */
    len = ( strlen((char *)data) < 45 ) ? 45 - strlen((char *)data) : 0;

   	fprintf(dbgFile, "%*s%*s:%s", 45 - len, data, len, pFunc, pBuf);
   	fflush(dbgFile);
}

/**************************************************************************//**
*
* dbgAssertInfo - handle the dbgAssert macro call
*
* DESCRIPTION
* Handle the dbgAssert and print out a message.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
dbgAssertInfo( c8_t * expr, c8_t * pFunc, c8_t * pFile, const int32_t line)
{
    c8_t	* p;

    p = (c8_t *)strrchr(pFile, '\\');
    if ( p == NULL )
        p = (c8_t *)strrchr(pFile, '/');

	fprintf(dbgFile, "\n*** Assertion: (%s) failed in %s:%s at %d\n", expr,
				(!p) ? pFile : ++p, pFunc, line);
   	fflush(dbgFile);

   	exit(-1);
}
