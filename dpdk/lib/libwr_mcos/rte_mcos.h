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

/**
 * \file
 * MCOS header file with many inline functions and structures.
 *
 * Multi-core OS (MCOS) is a non-preemptive OS using a light weight user
 * level threading model. The basic design under the hood uses setcontext,
 * getcontext, makecontext and swapcontext to provide the light weight
 * threading model. Some times the threads are called fibers to be different
 * from pthreads or other threading models.
 *
 * No Fiber is ever preempted and the only time a reschedule can occur is
 * when a fiber or thread gives up control of the system via a delay,
 * yield or taking a semaphore that does not have a positive count. You
 * can inject a vector in to the MCOS instance much like a interrupt, but
 * the fiber you are injecting must be a high priority fiber and the system
 * must attempt to reschedule to allow the high priority fiber to run.
 *
 * The high priority fiber is created but is suspended and when you inject
 * the vector or interrupt into the MCOS system it makes the fiber ready
 * to be run by placing it on the ready queue when a reschedule occurs.
 *
 * The MCOS code is model on "Operating System Design the XINU approach"
 * by Dougles Comer. The big difference is XINU is preemptive and XINU
 * has a global arrays and variables. MCOS does not contain any global
 * variables or arrays as all of the data and structures required for
 * running an instance of MCOS is contained in the mInfo_t structure. This
 * allows the user to start up more then one MCOS in a single address
 * space. Being able to run more then one instance allows DPDK to have
 * a single instance of MCOS per logical core and run without interaction
 * between the logical cores, which would reduce performance.
 *
 * Most of the code for MCOS is located in one .c file and one .h called
 * rte_mcos.c and rte_mcos.h files. The rte_mcos.c contains routines like
 * mcos_resched() and mcos_spawn(), which are routines I felt needed to
 * be non-inline routines as they are not performance critical routines.
 * The rest of the APIs for MCOS are located in rte_mcos.h and most of the
 * APIs are inline functions as most of the routines are fairly simple
 * and inlining them makes some sense.
 *
 * MCOS APIs that are not inline functions are contained in mcos.c and each
 * function has a simple function header explaining the basic usage of the
 * functions. Please look in the code for details.
 *
 * The rest of the MCOS APIs are inline functions contained in the rte_mcos.h
 * header file. Please look at this file for more details of the functions
 * and usage.
 *
 * The MCOS system now contains a set of routines and a structure to be the
 * master of all of the MCOS instances. The mcos_master_t and mcos_instance_t
 * structures by using these structures the master can now monitor and track
 * MCOS instances in the system. Their is also a method to send an 8 byte message
 * between fibers of an instance and between instances of MCOS. We use a simple
 * 16 byte structure as a message. The method to transport the message is
 * a set of lockless rings from DPDK. To move messages between MCOS instances
 * requires a call to mcos_dispatch() routine on a regular interval.
 *
 * The first 8 bytes contains the destination and source addresses. The
 * address uses a 32 bit word to encode the 2 8 bit instance id, lcore id and
 * the 16 bit fiber pid ID. The next 8 bytes can be used by the developer
 * as they see fit. It can be a 64 bit pointer or just data encoded into
 * the word.
 *
 * To execute the example code in examples/mcos directory, you need to build
 * a default DPDK system with 'make install T=x86_64-default-linuxapp-gcc'.
 * I have not tested this version with icc compiler and the code will only
 * runs in Linux user space.
 *
 * Created by Keith Wiles @ Wind River Systems (2014)
 *
 * \li \#include \<ucontext.h\>
 * \li \#include \<assert.h\>
 * \li \#include "rte_mcos_types.h"
 * \li \#include "rte_mcos_queue.h"
 * \li \#include "rte_mcos_sema.h"
 * \li \#include "rte_mcos_mutex.h"
 * \li \#include "rte_mcos_debug.h"
 *
 */

#include <ucontext.h>
#include <assert.h>

// Defines to enable debug support options.
#define INCLUDE_LOG_SUPPORT		0
#define INCLUDE_ASSERT_SUPPORT	1
#define INCLUDE_DEBUG_SUPPORT 	0

#include "rte_mcos_types.h"
#include "rte_mcos_queue.h"
#include "rte_mcos_sema.h"
#include "rte_mcos_mutex.h"

#ifndef rte_mcos_H_
#define rte_mcos_H_										/**< Used to not have recursive includes */

#define MCOS_VERSION			"1.2.0"					/**< MCOS version number */
#define MCOS_COPYRIGHT_MSG		"Copyright (c) <2011-2013>, Wind River Systems, Inc."
#define MCOS_POWERED_BY_DPDK	"Powered by Intel® DPDK"

#define MCOS_MASTER_INFO		"Master-info"			/**< Master structure memory name */
#define MCOS_MASTER_RX_DATA		"Master-RX-Data"		/**< Master RX data memory name */

typedef enum {
	FCB_EMPTY = -1,										/**< FCB is empty */
	FCB_FREE = 0,										/**< FCB is free */
	FCB_CURR,											/**< Current FCB state */
	FCB_READY,											/**< FCB is ready */
	FCB_WAITING,										/**< FCB is waiting */
	FCB_MUTEX_WAIT,										/**< FCB is waiting on a mutex */
	FCB_DEAD,											/**< FCB is dead */
	FCB_SUSPEND,										/**< FCB is suspended */
	FCB_SLEEP,											/**< FCB is a sleep */
	FCB_UNKNOWN											/**< Unknown FCB state */
} fcb_state_t;

#define DEFAULT_MCOS_TICK_RATE	RTE_MCOS_TICK_RATE		/**< Default tick rate value */

// Force alignment to cache line.
#ifndef CACHE_LINE_SIZE
#define CACHE_LINE_SIZE				64					/**< Default cache line size */
#endif

#define __cache_aligned		__attribute__((__aligned__(CACHE_LINE_SIZE)))	/**< Force the structure to a cache aligment */
#define roundup(x, y)		((x + (y - 1)) / y)								/**< Round up a value */
#define round64(x)			((x + (sizeof(int64_t) - 1)) / sizeof(int64_t))	/**< Round up to a 64 bit alignment */
#define mk_string(x)		#x												/**< Convert a value or variable to a string */

enum {
		MAX_NUM_MCOS		= RTE_MAX_NUM_MCOS,			/**< Max number of MCOS instances in the system */

		// These are the defaults if you pass in zero to mcos_initialize().
		DEFAULT_NUM_FIBERS	= 64,						/**< Default number of fibers */
		DEFAULT_NUM_SEMAS	= 128,						/**< default number of semaphores */
		DEFAULT_NUM_MUTEXS  = 128,						/**< default number of mutexs */

		MINSHORT			= 0x0000,					/**< min short value */
		MAXSHORT			= 0xFFFF,					/**< Max short value */

		NUM_EXTRA_QUEUES	= 4,						/**< One for each rdy, clock, ... queue */
		NUM_INTERNAL_QUEUES = (NUM_EXTRA_QUEUES * 2),	/**< Number of internal queues */
		MAX_VECTORS			= 16,						/**< Vectors */
		MAX_NAME_LENGTH		= 16,						/**< Max size of name strings */
		STACK_SHIFT			= sizeof(int64_t),			/**< 8 bytes multiples */
		DEFAULT_STACK_SIZE	= RTE_MCOS_DEFAULT_STACK_SIZE,	/**< Default stack size */
		DEFAULT_STACK_SHIFT	= (DEFAULT_STACK_SIZE >> STACK_SHIFT),
		NULLPROC_PID		= 0,						/**< NULLPROC pid id */
		NULLPROC_PRIO		= 0,						/**< NULLPROC priority value */
		CLOCK_VECTOR		= 0,						/**< Clock is the first vector number */
		MIN_PRIO			= (MINSHORT + 1),			/**< Min priority value. */
		MAX_PRIO			= (MAXSHORT - 1),			/**< Max priority value. */
		RESCHED_NO			= 0,						/**< Do not reschedule flag */
		RESCHED_YES			= 1,						/**< Do reschedule flag */
		E_EMPTY				= -1,						/**< queue is empty */
		E_TRUE				= 1,						/**< true value */
		E_FALSE				= 0,						/**< false value */
		E_OK				= 0,						/**< OK return value */
		E_ERROR				= -1,						/**< Error return value */
		E_WOULD_WAIT		= -2,						/**< Would wait return value */
		E_SEM_INVALID		= -3,						/**< Invalid semaphore return value */

		NB_SOCKETS			= 8,
		SOCKET0				= 0							/**< Socket ID value for allocation */
};

/**
 * Function returning string of version number: "- Version:x.y.x (DPDK-x.y.z)"
 * @return
 *     string
 */
static inline const char * mcos_dpdk_version(void) {
	return "Ver:"MCOS_VERSION"(DPDK-"
			RTE_STR(RTE_VER_MAJOR)"."
			RTE_STR(RTE_VER_MINOR)"."
			RTE_STR(RTE_VER_PATCH_LEVEL)")";
}

/**
 * Function returning string for Copyright message."
 * @return
 *     string
 */
static inline const char *
mcos_copyright(void) {
	return MCOS_COPYRIGHT_MSG;
}

#define E_NOOP				do { } while((0))			/**< Noop instruction that gets optimized out of the code. */

#if (INCLUDE_DEBUG_SUPPORT == 1)
#define isbadpid(x, _ret)	do { /**< Is this a bad pid index value */ \
								if ( (x) >= mInfo->num_fibers ) { \
									dbgPrintf("*** Bad pid %d >= %d\n", x, mInfo->num_fibers); \
									return _ret; \
								} \
							} while((0))
#define isbadq(x, _ret)		do { /**< Is this a bad queue index value */ \
								if ( (x) >= mInfo->num_queues ) { \
									dbgPrintf("*** Bad queue %d >= %d\n", x, mInfo->num_queues); \
									return _ret; \
								} \
							} while((0))
#define isbadsem(x, _ret)	do { /**< Is this a bad semaphore index value */ \
								if ( (x) >= mInfo->num_semas ) { \
									dbgPrintf("*** Bad sem %d >= %d\n", x, mInfo->num_semas); \
									return _ret; \
								} \
							} while((0))
#else
#define isbadpid(x, _ret)	E_NOOP				/**< Is this a bad pid index value */
#define isbadq(x, _ret)		E_NOOP				/**< Is this a bad queue index value */
#define isbadsem(x, _ret)	E_NOOP				/**< Is this a bad semaphore index value */
#endif

// Typedefs for the structures below.
typedef int32_t		mcos_once_t;				/**< 32bit value for once value */

typedef union {
	uint8_t		data8[8];						/**< 8 bit data values */
	uint16_t	data16[4];						/**< 16 bit data values */
	uint32_t	data32[2];						/**< 32 bit data value in union */
	uint64_t	data64;							/**< 64 bit data value in union */
	void	  * pdata;							/**< pointer value in union */
} data_u;										/**< data structure */

#define ALLOCATED_MEMORY_FLAG					0x00000001

typedef struct tqueue_s {
	struct timer_s	  * next;					/**< Timer link list pointer */
	uint32_t			timeout;				/**< Time out value in ticks */
	uint32_t			pad;
	data_u				tdata;					/**< Timer data area */
} tqueue_t;

typedef union instance_data_u {
	struct {
		uint8_t			idx;					/**< MCOS Instance index */
		uint8_t			lcore;					/**< lcore value */
		uint16_t		pid;					/**< PID for fiber in MCOS instance */
	} s;										/**< Instance structure */
	struct {
		uint16_t		id;
		uint16_t		pid;
	} w;
	uint32_t			data;					/**< Union of the instance structure */
} mcos_addr_t;

typedef struct mcos_data_s {
	mcos_addr_t			dst_id;					/**< Instance ID for destination */
	mcos_addr_t			src_id;					/**< Instance ID of source */
	uint64_t			info;					/**< Data for message */
} mcos_data_t;									/**< Data structure used to move messages between instances */

typedef struct mcos_config_s {
	void			  * master;					/**< Master pointer for reference */
	void			  * instance;				/**< MCOS Instance pointer */
	void			  * arg;					/**< argument from caller */
	uint16_t			num_fibers;				/**< configured number of fibers */
	uint16_t			num_semas;				/**< configured number of semaphores */
	uint16_t			num_mutexs;				/**< configured number of mutexs */
	uint16_t			lcore;					/**< lcore instance is running on */
} mcos_config_t;								/**< Configuration information for each MCOS instance */

typedef struct mcos_instance_s {
	struct rte_msg	  * msg_ring;				/**< Ring end point for instance. */
	struct mcos_master_s * master;				/**< Master reference pointer */
	struct mInfo_s	  * mInfo;					/**< mInfo_t pointer value for the instance */
	char				name[MAX_NAME_LENGTH];	/**< instance name for lookup later. */
	mcos_addr_t			instance_id;			/**< (instance idx << 24) | (lcore ID << 16) value */
	mcos_config_t		config;					/**< temp storage of the configuration */
} mcos_instance_t __cache_aligned;				/**< Instance information for each MCOS */

#define MASTER_MSG_COUNT		RTE_MASTER_MSG_COUNT
typedef struct mcos_master_s {
	rte_spinlock_t		lock;					/**< Master spinlock structure */
	uint16_t			instance_count;			/**< Current instance count. */
	uint16_t			curr_index;				/**< Current index for mcos_dispatch() */
	uint16_t			pad0;
	uint16_t			msg_cnt;				/**< Number of entries in the ring */
	mcos_data_t		  * msg_buff;				/**< Dispatch data buffer */
	mcos_instance_t		instance[MAX_NUM_MCOS];	/**< Pointer array for each instance. */
	uint64_t			dispatch_sent;			/**< Number of sent messages to rings */
	uint64_t			dispatch_error;			/**< Number of errors in sending to a ring */
} mcos_master_t __cache_aligned;

typedef struct fcb_s {							/**< Fiber Control Block (FCB) */
	uint16_t			pid;					/**< FCB index value */
	uint16_t			priority;				/**< Priority of FCB */
	fcb_state_t			state;					/**< Free or used */
	uint16_t			stacksize;				/**< in 8 byte words */
	uint16_t			flags;					/**< Internal flags */
	uint16_t			semid;					/**< Semaphore we are waiting on */
	uint32_t			vector_arg;				/**< FCB vector argument */

	void			  * priv;					/**< Private data in caller context. */
	void			  * stack;					/**< Pointer to stack space */
	ucontext_t		  * uctx;					/**< allocated storage */
	struct rte_msg	  * msg_ring;				/**< Message Ring of messages for this fiber */
	tqueue_t		  * timer;					/**< Timer queue */
	char				name[MAX_NAME_LENGTH];	/**< Thread name */
} fcb_t __cache_aligned;

typedef struct mInfo_s {
	volatile uint32_t	flags;					/**< Flags for MCOS */

	int16_t				num_fibers;				/**< Number of fibers */
	int16_t				num_queues;				/**< Number of queues */
	int16_t				num_semas;				/**< Number of semaphores */
	int16_t				num_mutexs;				/**< Number of mutex */

	int16_t				nextfcb;				/**< Next FCB structure */
	int16_t				nextqueue;				/**< Next free queue */
	int16_t				nextsema;				/**< Next free semaphore */
	int16_t				nextmutex;				/**< Next free mutex */

	int16_t				rdyhead;				/**< Ready queue head */
	int16_t				rdytail;				/**< Ready queue tail */
	int16_t				currpid;				/**< Current PID value */
	int16_t				lcore;					/**< lcore instance is running on */

	uint64_t			scheduler_cnt;			/**< Number of scheduler loops */
	uint64_t			nullproc_loops;			/**< Number of times nullproc loops calling mcos_resched() */

	rte_atomic32_t		vector_bitmap;			/**< bitmap of active vectors */
	int16_t			  * vector;					/**< Vectors used in signal */

	ucontext_t		  * done;					/**< Done context */
	fcb_t			  * curr;					/**< Current running fiber */
	fcb_t			  * fcbs;					/**< Place holder for the FCB structures */
	qentry_t		  * q;						/**< All of the queues */
	sentry_t		  * sem;					/**< Semaphore structure array */
	mentry_t		  * mutex;					/**< Mutex structure array. */

	uint32_t			z[4];					/**< Random number seed values. */
	uint32_t			tickrate;				/**< Current tick rate for the clock */
	int16_t				sleepq;					/**< Index value for the sleepq */
	int16_t				slnempty;				/**< Sleep queue not empty flag */
	int16_t				defclk;					/**< Defer the clock tick if non zero. */
	int16_t				clkdiff;				/**< Number of clock ticks deferred */
	qentry_t		  * sltop;					/**< Point to the top entry in the sleepq */
	uint64_t			clk;					/**< Clock count value incremented for each clock tick */
	uint64_t			hz;						/**< Current HZ value for the HPET */

	mcos_master_t	  * master;					/**< Pointer to the master structure */
	mcos_instance_t	  * instance;				/**< Pointer to the MCOS instance structure */
	uint64_t			elapsedTime;			/**< single use elapsed time value for viper */
	struct rte_timer	timer0;					/**< Timer timeout structure */
} mInfo_t __cache_aligned;

typedef	void (*func_t)(mInfo_t * mInfo, void * arg1);	/**< typedef for fiber entry point */

/****************************************************************************/

#define ONCE_FLAG		0x8000					/**< Execute the given function only once */
#define STOP_FLAG		0x4000					/**< Stop the MCOS instance */
#define INIT_FLAG		0x0001					/**< MCOS instance is inited. */
#define CLOCK_FLAG		0x0002					/**< Clock flag */

/****************************************************************************/

#include "rte_mcos_debug.h"

// Part of the MCOS API plus the inline functions below.
/**************************************************************************//**
*
* Initialize the mcos system and start the scheduler running.
*
* \param  ret Pointer to pointer to return MCOS structure pointer.
* \param  config Configuration structure to MCOS.
*
* \returns E_ERROR or E_OK.
*/

extern int32_t	mcos_initialize( mInfo_t ** ret, mcos_config_t * config );

/**************************************************************************//**
*
* Internal routine to spawn a new fiber with the given arguments.
*
* \param  mInfo Pointer to MCOS structure.
* \param  name Pointer to name of MCOS instance.
* \param  priority Priority value for fiber higer the number the high the priority.
* \param  stack Pointer to a stack memory area.
* \param  stacksize Amount of bytes for the stack if zero then use default value.
* \param  start_routine Routine to call to start the fiber. Prototype for the routine
* is `void start_routine(void * mInfo, void * arg);`
* \param  arg Argument for the start_routine when called.
*
* \returns return the pid value or E_ERROR on error
*/

extern int32_t __spawn(mInfo_t * mInfo, c8_t * name, int16_t prio, char * stack,
						int32_t stacksize, func_t func, void * arg, fcb_t ** ppFcb);

/**************************************************************************//**
*
* Spawn a new fiber with the given arguments.
*
* \param  mInfo Pointer to MCOS structure.
* \param  name Ponter to name of MCOS instance.
* \param  priority Priority value for fiber higer the number the high the priority.
* \param  stacksize Amount of bytes for the stack if zero then use default value.
* \param  start_routine Routine to call to start the fiber. Prototype for the routine
* is `void start_routine(void * mInfo, void * arg);`
* \param  arg Argument for the start_routine when called.
*
* \returns return the pid value or E_ERROR on error
*/

extern int16_t	mcos_spawn( mInfo_t * mInfo, c8_t * name, int16_t priority,
							uint32_t stacksize,	func_t start_routine, void * arg );

/**************************************************************************//**
*
* This routine is called back during the initialization  of MCOS.
*
* \param  mInfo Pointer to MCOS structure.
* \param  arg Argument for the start_routine when called.
*
* \returns N/A
*/

extern void		mcos_init_callback(void * mInfo, void * arg);

/**************************************************************************//**
*
* Reschedule the current fiber for the next highest fiber priority.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns N/A
*/

extern void		mcos_resched(mInfo_t * mInfo);

/**************************************************************************//**
*
* Set the interrupt or signal in the bitmap.
*
* \param  mInfo Pointer to MCOS structure.
* \param  v is the vector value 0-31.
* \param  arg is the argument to be passed to the routine.
*
* \returns N/A
*/

extern void		mcos_interrupt(mInfo_t * mInfo, int32_t v, int32_t arg);

/**************************************************************************//**
*
* Launch the first MCOS thread and use the mcos_launch running on the lcore
* to initialize the system. The mcos_launch() will also call the callback
* routine in the application to finish initializing the application threads.
*
* \param  master Pointer to master MCOS structure.
* \param  config Pointer to the configuration structure.
* \param  lcore The lcore to launch the routine on.
*
* \returns E_ERROR or E_OK
*/
extern int32_t	mcos_remote_launch(mcos_master_t * master, mcos_config_t * config, uint32_t lcore);

// Master routines
/**************************************************************************//**
*
* Process any messages and dispatch them to the correct process
*
* \param  master Pointer to master MCOS structure.
*
* \returns E_ERROR or Number of receives
*/

extern void	mcos_msg_dispatch(mcos_master_t * master);

/****************************************************************************/

/**************************************************************************//**
*
* Return the number of lcores used and fill in the index array of lcore ids.
* The master and map are returned if not NULL pointers. The map array only has
* the non-master lcores listed.
*
* \returns number of lcores enabled not include master.
*/
static __inline__ uint32_t mcos_map_lcores(uint8_t * map, uint8_t * master) {
	uint32_t		lcore, i, mcore;

	mcore = rte_get_master_lcore();
	if ( master )
		*master = (uint8_t)mcore;

	if ( map )
		memset(map, 0xff, RTE_MAX_LCORE);

    // Count the number of lcores being used.
    for(lcore = 0, i = 0; i<RTE_MAX_LCORE; i++) {
        if ( rte_lcore_is_enabled(i) && (i != mcore) ) {
        	if ( map )
        		map[lcore++] = (uint8_t)i;
        	else
        		lcore++;
        }
    }

    return lcore;
}

/**************************************************************************//**
*
* Return the version string for mcos.
*
* \returns Version string pointer.
*/

static __inline__ c8_t * mcos_version(void) {
	return MCOS_VERSION;
}

/**************************************************************************//**
*
* Return the current tick rate used by MCOS.
*
* \returns tick rate value
*/

static __inline__ uint32_t mcos_get_tickrate(void) {
	return DEFAULT_MCOS_TICK_RATE;
}

/**************************************************************************//**
*
* Return the fcb_t pointer to the current fiber running.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns NULL or pointer to fcb_t.
*/

static __inline__ fcb_t * mcos_self(mInfo_t * mInfo) {
	return mInfo->curr;
}

/**************************************************************************//**
*
* Return current PID ID.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns Current pid
*/

static __inline__ uint16_t mcos_currpid(mInfo_t * mInfo) {
	return mInfo->currpid;
}

/**************************************************************************//**
*
* Force a reschedule or yield the current fiber.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns N/A
*/

static __inline__ void mcos_yield(mInfo_t * mInfo) {
	mcos_resched(mInfo);
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_index -
* 		return the index value from the instance_id
* \param addr The instance address value to parse.
*
* \returns The instance index value in the instance ID.
*
*/

static __inline__ uint8_t mcos_instance_index(mcos_addr_t addr) {
	return addr.s.idx;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_lcore -
* 		return the lcore value from the instance_id
* \param addr The instance address value to parse.
*
* \returns The lcore value in the instance ID.
*
*/
static __inline__ uint8_t mcos_instance_lcore(mcos_addr_t addr) {
	return addr.s.lcore;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_pid -
* 		return the pid value from the instance_id
* \param addr The instance address value to parse.
*
* \returns The instance pid value in the instance ID.
*
*/
static __inline__ uint16_t mcos_instance_pid(mcos_addr_t addr) {
	return addr.s.pid;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_pid_self -
* 		return the pid value of self.
* \param mInfo pointer to mInfo self.
*
* \returns The upper 16 bits of the instance value containing the instance
* index and lcore value.
*
*/
static __inline__ uint16_t mcos_instance_pid_self(mInfo_t * mInfo) {
	return mInfo->instance->instance_id.w.pid;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_id -
* 		return the index and lcore value
* \param addr The instance address value to parse.
*
* \returns The upper 16 bits of the instance value containing the instance
* index and lcore value.
*
*/
static __inline__ uint16_t mcos_instance_id(mcos_addr_t addr) {
	return addr.w.id;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_id_self -
* 		return the index and lcore value
* \param mInfo Pointer to mInfo self.
*
* \returns The upper 16 bits of the instance value containing the instance
* index and lcore value.
*
*/
static __inline__ uint16_t mcos_instance_id_self(mInfo_t * mInfo) {
	return mInfo->instance->instance_id.w.id;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_mk_instance -
* 		create a instance_id value.
* \param idx The instance index value.
* \param lcore The instance lcore value.
* \param pid The instance pid value.
*
* \returns The instance address 32 bit value encoded with the paramaters.
*
*/
static __inline__ mcos_addr_t mcos_mk_instance(uint8_t idx, uint8_t lcore, uint16_t pid) {
	mcos_addr_t		addr;

	addr.s.idx		= idx;
	addr.s.lcore	= lcore;
	addr.s.pid		= pid;
	return addr;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_instance_self -
* 		return the instance_id of self.
* \param mInfo pointer to mInfo.
*
* \returns The instance id value.
*
*/
static __inline__ mcos_addr_t mcos_instance_self(mInfo_t * mInfo) {
	mcos_addr_t		addr;

	addr.w.id		= mcos_instance_id_self(mInfo);
	addr.w.pid		= mcos_currpid(mInfo);
	return addr;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_mk_instance_id -
* 		create a instance_id value.
* \param idx The instance index value.
* \param pid The instance pid value.
*
* \returns The instance address 32 bit value encoded with the paramaters.
*
*/
static __inline__ mcos_addr_t mcos_mk_instance_id(uint16_t id, uint16_t pid) {
	mcos_addr_t		addr;

	addr.w.id		= id;
	addr.w.pid		= pid;
	return addr;
}

/**************************************************************************//**
*
* Set of routines to manage the instance_id value.
*
* mcos_add_instance_pid -
* 		Update the pid value in the instance_id
* \param addr The instance address value.
* \param pid The pid value to encode in the the instance ID.
*
* \returns The instance address encoded in 32 bit instance ID.
*
*/
static __inline__ mcos_addr_t mcos_add_instance_pid(mcos_addr_t addr, uint16_t pid) {
	addr.w.pid = pid;
	return addr;
}

/**************************************************************************//**
*
* Create the instance ID string.
*
* mcos_instance_tostring -
* 		Create the instance ID string.
* \param addr The instance address value.
* \param str Pointer string to place the information.
* \param len Max length of string.
*
* \returns a Null terminated string with instance information.
*
*/
static __inline__ char * mcos_instance_tostring(mcos_addr_t addr) {
	static char str[32];
	snprintf(str, sizeof(str), "%02x:%02x:%04x",
			mcos_instance_index(addr), mcos_instance_lcore(addr), mcos_instance_pid(addr));
	return str;
}

/**************************************************************************//**
*
* Initialize the mcos master system.
*
* \returns NULL on ERROR or pointer to master structure.
*/

static __inline__ mcos_master_t * mcos_master_initialize(void) {
	mcos_master_t	  * master;

	master = (mcos_master_t *)rte_zmalloc(MCOS_MASTER_INFO, sizeof(mcos_master_t), 0);
	if ( master ) {
		master->msg_cnt = MASTER_MSG_COUNT;
		master->msg_buff = (mcos_data_t *)rte_zmalloc(MCOS_MASTER_RX_DATA,
													(master->msg_cnt * sizeof(mcos_data_t)), 0);
		if ( master->msg_buff == NULL ) {
			rte_free(master);
			master = NULL;
		}
	}
	return master;
}

/**************************************************************************//**
*
* Allocate an instance structure from the master list of instance structures.
*
* \param  master Pointer to the master structure.
*
* \returns E_ERROR or the instance pointer
*/

static __inline__ mcos_instance_t * mcos_alloc_instance(mcos_master_t * master) {
	int32_t				idx;
	mcos_instance_t	  * instance;

	rte_spinlock_lock(&master->lock);
	for(idx = 0; likely(idx < MAX_NUM_MCOS); idx++) {
		if ( (instance = &master->instance[idx])->master == NULL ) {
			instance->master		= master;
			instance->instance_id	= mcos_mk_instance(idx, rte_lcore_id(), 0);
			master->instance_count++;
			rte_spinlock_unlock(&master->lock);
			return instance;
		}
	}
	rte_spinlock_unlock(&master->lock);

	return NULL;
}

/**************************************************************************//**
*
* Update the MCOS instance information to the master list and return the id.
* Must be called in the context of the running lcore to work correctly.
*
* \param  mInfo Pointer to MCOS structure.
* \param  name Pointer to the name to be updated.
* \param  ring_size Number of 64bit entries in the ring.
*
* \returns instance address
*/

static __inline__ mcos_addr_t mcos_initialize_instance( mInfo_t * mInfo, char * name, int32_t ring_size ) {

	mcos_instance_t	  * instance;
	mcos_addr_t			addr;
	char				ring_str[16];

	dbgAssert( name != NULL );
	dbgAssert( mInfo != NULL );

	instance	= mInfo->instance;
	dbgAssert( instance != NULL );

	rte_spinlock_lock(&instance->master->lock);
	strncpy(instance->name, name, MAX_NAME_LENGTH);

	addr = instance->instance_id;
	addr.w.pid = mcos_instance_pid(addr);
	snprintf(ring_str, sizeof(ring_str)-1, "MSG_%s", mcos_instance_tostring(addr));

	instance->msg_ring = rte_msg_create(ring_str, ring_size, sizeof(mcos_data_t),
			rte_lcore_to_socket_id(rte_lcore_id()), MSG_F_SC_DEQ);	// MP and SC ring

	dbgAssert(instance->msg_ring != NULL);

	rte_spinlock_unlock(&instance->master->lock);
	rte_wmb();

	return addr;
}

/**************************************************************************//**
*
* Find a instance id of a fiber matching the given name.
*
* \param master Pointer to the master structure.
* \param name Search name.
*
* \returns E_ERROR or instance_id
*/

static __inline__ uint32_t mcos_lookup_fiber_by_name(mcos_master_t * master, char * name)
{
	mcos_instance_t	  * instance;
	mInfo_t			  * mInfo;
	fcb_t			  * pFcb;
	int32_t				id, i;

	// Loop looking for valid instances with a valid name.
	for(id = 0; likely(id < MAX_NUM_MCOS); id++) {
		if ( unlikely((instance = &master->instance[id])->mInfo == NULL) )
			continue;
		mInfo = instance->mInfo;

		for(i = 0; likely(i < mInfo->num_fibers); i++) {
			pFcb = &mInfo->fcbs[i];
			if ( (pFcb->state != FCB_FREE) && (pFcb->msg_ring != NULL) ) {
				if ( strcmp(name, pFcb->name) == 0 )
					return instance->instance_id.data | pFcb->pid;
			}
		}
	}

	return E_ERROR;
}
/**************************************************************************//**
*
* Find the MCOS instance information in the master list and return the instance id.
*
* \param  master Pointer to the master structure.
* \param  mInfo Pointer to the MCOS structure.
*
* \returns E_ERROR or the instance id.
*/

static __inline__ mcos_instance_t * mcos_find_instance( mcos_master_t * master, mInfo_t * mInfo ) {
	int32_t				id;
	mcos_instance_t	  * instance;

	dbgAssert( master != NULL );
	dbgAssert( mInfo != NULL );

	for(id = 0; likely(id < MAX_NUM_MCOS); id++) {
		instance = &master->instance[id];
		if ( (instance->master != NULL) && (instance->mInfo == mInfo) )
			rte_spinlock_unlock(&master->lock);
			return instance;
	}

	return NULL;
}

/**************************************************************************//**
*
* Find the MCOS instance information in the master list and return the id.
*
* \param  master Pointer to master MCOS structure.
* \param  name Pointer to the name to be found.
*
* \returns E_ERROR or the instance id.
*/

static __inline__ mcos_instance_t * mcos_find_instance_by_name( mcos_master_t * master, char * name ) {
	uint32_t			i;
	mcos_instance_t	  * instance;

	dbgAssert( master != NULL );

	for(i = 0; likely(i < MAX_NUM_MCOS); i++) {
		instance = &master->instance[i];
		if ( (instance->master != NULL) && (strcmp(instance->name, name) == 0) )
			return instance;
	}
	return NULL;
}

/**************************************************************************//**
*
* Delete the instance from the master list.
*
* \param  instance Pointer to master instance structure.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_del_instance( mcos_instance_t * instance ) {
	mcos_master_t	  * master;

	dbgAssert( instance != NULL );

	master = instance->master;
	dbgAssert( master != NULL );

	rte_spinlock_lock(&master->lock);
	master->instance_count--;
	instance->mInfo = NULL;
	instance->master = NULL;
	rte_spinlock_unlock(&master->lock);

	return E_OK;
}

/**************************************************************************//**
*
* Return the mInfo_t structure pointer for the given instance id;
*
* \param  instance Pointer to master instance structure.
*
* \returns E_ERROR or the instance id.
*/

static __inline__ mInfo_t * mcos_get_instance_info( mcos_instance_t * instance ) {
	mInfo_t	  		  * mInfo = NULL;

	dbgAssert(instance != NULL );

	if ( likely(instance->master != NULL) )
		mInfo = instance->mInfo;

	return mInfo;
}

/**************************************************************************//**
*
* Return the name pointer string for the given lcore;
*
* \param  instance Pointer to master instance structure.
* \param  name Pointer the the name to find.
* \param  name_len Size of the name array or zero to use the default size.
*
* \returns NULL or pointer to name
*/

static __inline__ void mcos_get_instance_name( mcos_instance_t * instance, char * name, int32_t name_len ) {

	dbgAssert( instance != NULL );
	dbgAssert( name != NULL );

	strncpy(name, instance->name, (name_len == 0)? MAX_NAME_LENGTH : name_len);
}

/**************************************************************************//**
*
* Create a ring for this FCB to use for messages.
*
* \param  mInfo Pointer to MCOS structure.
* \param  ring_size Size of ring structure to allocate.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_msg_create( mInfo_t * mInfo, int32_t ring_size ) {
	fcb_t	  * fcb;
	mcos_addr_t	addr;

	dbgAssert( mInfo != NULL );

	rte_spinlock_lock(&mInfo->master->lock);
	fcb = mcos_self(mInfo);
	if ( likely(fcb->msg_ring == NULL) ) {
		char	ring_str[24];

		addr		= mInfo->instance->instance_id;
		addr.w.pid	= fcb->pid;
		snprintf(ring_str, sizeof(ring_str)-1, "mg_%s", mcos_instance_tostring(addr));

		fcb->msg_ring = rte_msg_create(ring_str, ring_size, sizeof(mcos_data_t),
				rte_lcore_to_socket_id(rte_lcore_id()), MSG_F_SC_DEQ);
		dbgAssert(fcb->msg_ring != NULL);
	}
	rte_spinlock_unlock(&mInfo->master->lock);

	return (fcb->msg_ring == NULL)? E_ERROR : E_OK;
}

/**************************************************************************//**
*
* Send an object pointer to an instance.
*
* \param  instance Pointer to master instance structure.
* \param  dst_id The destination 32bit instance ID value.
* \param  data pointer to a mcos_data_t structure.
*
* \returns Number of message sent.
*/

static __inline__ int32_t mcos_send_instance( mcos_instance_t * instance, mcos_addr_t dst_id, mcos_data_t * data ) {
	struct rte_msg	  * rg;
	int32_t				ret = 0;
	uint8_t				id = dst_id.s.idx;
	mcos_master_t	  * master;

	dbgAssert(instance != NULL);
	dbgAssert(id < MAX_NUM_MCOS);

	master = instance->master;
	dbgAssert( master != NULL );

	data->dst_id	= dst_id;
	data->src_id	= instance->instance_id;

	if ( likely((rg = master->instance[id].msg_ring) != NULL) ) {
		dbgAssert( mcos_instance_id(master->instance[id].instance_id) == mcos_instance_id(dst_id) );

		ret = rte_msg_enqueue_burst(rg, (void *)data, 1);
	}

	return ret;
}

/**************************************************************************//**
*
* Receive an object pointer from an instance.
*
* \param  instance Pointer to master instance structure.
* \param  data pointer to a mcos_data_t structure.
*
* \returns Number of messages received.
*/

static __inline__ int32_t mcos_recv_instance( mcos_instance_t * instance, mcos_data_t * data ) {
	struct rte_msg	  * rg;
	int32_t				ret = 0;

	dbgAssert( instance != NULL );

	if ( likely((rg = instance->msg_ring) != NULL) )
		ret = rte_msg_dequeue_burst(rg, (void *)data, 1);

	return ret;
}

/**************************************************************************//**
*
* Send an object pointer to an instance.
*
* \param  instance Pointer to master instance structure.
* \param  dst_id The destination 32bit instance ID value.
* \param  data pointer to a mcos_data_t structure.
* \param  cnt Number of data objects to send.
*
* \returns Number of messages sent.
*/

static __inline__ int32_t mcos_send_instance_burst( mcos_instance_t * instance,
		mcos_addr_t dst_id, mcos_data_t * data, int32_t cnt ) {
	struct rte_msg	  * rg;
	int32_t				ret = 0, i;
	uint8_t				id;
	mcos_data_t		  * p;
	mcos_addr_t			src;
	mcos_master_t	  * master;

	dbgAssert(instance != NULL);

	id = mcos_instance_index(dst_id);
	dbgAssert(id < MAX_NUM_MCOS);

	master = instance->master;
	dbgAssert( master != NULL );

	src = mcos_instance_self(instance->mInfo);

	for(i = 0, p = data; i < cnt; i++, p++) {
		p->dst_id	= dst_id;
		p->src_id	= src;
	}

	if ( likely((rg = master->instance[id].msg_ring) != NULL) ) {
		dbgAssert( mcos_instance_id(master->instance[id].instance_id) == mcos_instance_id(data->dst_id) );

		ret = rte_msg_enqueue_burst(rg, (void *)data, cnt);
	}

	return ret;
}

/**************************************************************************//**
*
* Receive an object pointer from an instance.
*
* \param  instance Pointer to master instance structure.
* \param  data pointer to a mcos_data_t structure.
* \param  cnt Number of data object to receive
*
* \returns number of object received.
*/

static __inline__ int32_t mcos_recv_instance_burst( mcos_instance_t * instance,
		mcos_data_t * data, int32_t cnt ) {
	struct rte_msg	  * rg;
	int32_t				ret = 0;

	dbgAssert( instance != NULL );

	if ( likely((rg = instance->msg_ring) != NULL) )
		ret = rte_msg_dequeue_burst(rg, (void *)data, cnt);

	return ret;
}

/**************************************************************************/

/**************************************************************************//**
*
* Send an object pointer to a fiber
*
* \param  mInfo Pointer to MCOS structure.
* \param  dst_id The destination 32bit instance ID value.
* \param  data pointer to a mcos_data_t structure.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_send_message( mInfo_t * mInfo, mcos_addr_t dst_id, mcos_data_t * data ) {
	int32_t		ret = 0;

	dbgAssert( mInfo != NULL );
	dbgAssert( mInfo->instance != NULL );

	// Test to see if the message is for a local fiber.
	if ( mcos_instance_id(mInfo->instance->instance_id) == mcos_instance_id(dst_id) ) {
		struct rte_msg	  * rg;
		fcb_t			  * fcb = &mInfo->fcbs[dst_id.s.pid];

		if ( likely((rg = fcb->msg_ring) != NULL) ) {
			data->dst_id	= dst_id;
			data->src_id	= mcos_instance_self(mInfo);

			ret = rte_msg_enqueue_burst(rg, (void *)data, 1);
		}
	} else	{	// Send to another MCOS instance.
		ret = mcos_send_instance(mInfo->instance, dst_id, data);
	}
	return ret;
}

/**************************************************************************//**
*
* Receive an object pointer from an instance.
*
* \param  mInfo Pointer to MCOS structure.
* \param  data pointer to a mcos_data_t structure.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_recv_message( mInfo_t * mInfo, mcos_data_t * data ) {
	struct rte_msg	  * rg;
	fcb_t			  * fcb;

	dbgAssert( mInfo != NULL );
	dbgAssert( data != NULL );

	fcb = mcos_self(mInfo);
	if ( likely((rg = fcb->msg_ring) != NULL) ) {
		if ( rte_msg_dequeue_burst(rg, (void *)data, 1) > 0 ) {
			dbgAssert( (mInfo->instance->instance_id.data || (mcos_currpid(mInfo)) == data->dst_id.s.pid) );
			return 1;
		}
	}

	return 0;
}

/**************************************************************************//**
*
* Send an list of data objects to the same destination address.
*
* \param  mInfo Pointer to MCOS structure.
* \param  dst_id The destination 32bit instance ID value.
* \param  data pointer to a mcos_data_t structure.
* \param  cnt Number of data objects to send.
*
* \returns Number of objects sent.
*/

static __inline__ int32_t mcos_send_message_burst( mInfo_t * mInfo, mcos_addr_t dst_id,
		mcos_data_t * data, int32_t cnt ) {
	int32_t				ret = 0;
	int32_t				i;

	dbgAssert( mInfo != NULL );
	dbgAssert( mInfo->instance != NULL );

	// Test to see if the message is for a local fiber.
	if ( mcos_instance_id(mInfo->instance->instance_id) == mcos_instance_id(dst_id) ) {
		struct rte_msg	  * rg;
		fcb_t			  * fcb = &mInfo->fcbs[dst_id.s.pid];

		if ( likely((rg = fcb->msg_ring) != NULL) ) {
			mcos_data_t	  * p = data;
			mcos_addr_t		src;

			src = mcos_mk_instance_id(mcos_instance_id(mInfo->instance->instance_id), mcos_currpid(mInfo));

			for ( i = 0; i < cnt; i++, p++ ) {
				p->dst_id	= dst_id;
				p->src_id	= src;
			}
			dbgAssert( mcos_instance_id(data->dst_id) == mcos_instance_id(data->src_id) );

			ret = rte_msg_enqueue_burst(rg, (void *)data, cnt);
		}
	} else	{	// Send to another MCOS instance.
		ret = mcos_send_instance_burst(mInfo->instance, dst_id, data, cnt);
	}
	return ret;
}

/**************************************************************************//**
*
* Receive an object pointer from an instance.
*
* \param  mInfo Pointer to MCOS structure.
* \param  data pointer to a mcos_data_t structure.
*
* \returns E_ERROR or Number of objects received.
*/

static __inline__ int32_t mcos_recv_message_burst( mInfo_t * mInfo, mcos_data_t * data, int32_t cnt ) {
	struct rte_msg	  * rg;
	fcb_t			  * fcb;
	int32_t				rx_cnt;

	dbgAssert( mInfo != NULL );
	dbgAssert( data != NULL );

	fcb = mcos_self(mInfo);
	if ( likely((rg = fcb->msg_ring) != NULL) ) {
		rx_cnt = rte_msg_dequeue_burst(rg, (void *)data, cnt);
		if ( likely( rx_cnt > 0) )
			return rx_cnt/2;
	}

	return 0;
}

/**************************************************************************//**
*
* This routine sets the seed value used by mcos_rand32().  If mcos_srand32() is
* then called with the same seed value, the sequence of pseudo-random numbers
* is repeated.  If mcos_rand32() is called before any calls to mcos_srand32() have
* been made, the same sequence shall be generated as when mcos_srand32() is
* first called with the seed value of 1.
*
* \param  mInfo Pointer to MCOS structure.
* \param  seed The send value for the random number generator.
*
* \returns N/A
*
* @see mcos_rand32()
*/

static __inline__ void  mcos_srand32( mInfo_t * mInfo, uint32_t seed ) {
    mInfo->z[0] = (uint32_t)seed;
    mInfo->z[1] = (uint32_t)seed + 7;
    mInfo->z[2] = (uint32_t)seed + 21;
    mInfo->z[3] = (uint32_t)seed + 71;

    dbgPrintf("Seed %08x\n", seed);
}

/**
   32-bits Random number generator U[0,1): lfsr113
   Author: Pierre L'Ecuyer,
   Source: http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme2.ps
   ---------------------------------------------------------
*/
static __inline__ uint32_t __lfsr113_Bits (mInfo_t * mInfo)
{
   uint32_t z1 = mInfo->z[0], z2 = mInfo->z[1], z3 = mInfo->z[2], z4 = mInfo->z[3];
   uint32_t b;

   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27;
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;

   mInfo->z[0] = z1;
   mInfo->z[1] = z2;
   mInfo->z[2] = z3;
   mInfo->z[3] = z4;

   return (z1 ^ z2 ^ z3 ^ z4);
}

/**************************************************************************//**
*
* This routine generates a pseudo-random integer between 0 and RAND_MAX.
* The seed value for mcos_rand32() can be reset with mcos_srand32().
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns A pseudo-random integer.
*
* \see mcos_srand32()
*/

static __inline__ uint32_t mcos_rand32 (mInfo_t * mInfo) {
    return __lfsr113_Bits(mInfo);
}

/**************************************************************************//**
*
* Return the state of a given fiber as a string.
*
* \param  state The state value to be returned as a string.
*
* \returns String pointer
*/

static __inline__ c8_t * mcos_state(int16_t state)
{
	static const char * fcb_states[FCB_UNKNOWN+1] = {
			"Free",
			"Current",
			"Ready",
			"Waiting",
			"Mutex Wait",
			"Dead",
			"Suspend",
			"Sleeping",
			"Unknown"
	};

	if ( state >= FCB_UNKNOWN )
		return fcb_states[FCB_UNKNOWN];
	else
		return fcb_states[state];
}

/**************************************************************************//**
* Atomic setting if a given bit in a 32bit bitmap.
* Set a bit in a bitmap using atomic routines.
*
* \param  vbitmap The 32bit atomic word to set the bit into.
* \param  bit The bit location to set in the vbitmap word.
*
* \returns N/A
*/

static __inline__ void mcos_set_bit(rte_atomic32_t * vbitmap, int32_t bit) {
	uint32_t bitmap = rte_atomic32_read(vbitmap);

	while( rte_atomic32_cmpset((uint32_t *)vbitmap, bitmap, (bitmap | (1 << bit))) == 0 ) {
		bitmap = rte_atomic32_read(vbitmap);
	}
}

/**************************************************************************//**
* Atomic clearing of a given bit in a 32bit bitmap.
* Clear a bit in a bitmap using atomic routines.
*
* \param  vbitmap The 32bit atomic word to set the bit into.
* \param  bit The bit location to clear in the vbitmap word.
*
* \returns N/A
*/

static __inline__ void mcos_clr_bit(rte_atomic32_t * vbitmap, int32_t bit) {
	uint32_t bitmap = rte_atomic32_read(vbitmap);
	while( rte_atomic32_cmpset((uint32_t *)vbitmap, bitmap, (bitmap & ~(1 << bit))) == 0 ) {
		bitmap = rte_atomic32_read(vbitmap);
	}
}

/**************************************************************************//**
*
* Return the key value for the given queue entry.
*
* \param  mInfo Pointer to MCOS structure.
* \param  head The head of the queue to grab the key from.
*
* \returns NULL or pointer to fcb_t.
*/

static __inline__ int32_t mcos_firstkey(mInfo_t * mInfo, uint16_t head) {
	qentry_t	  * q = mInfo->q;

	isbadq(head, E_ERROR);

	return q[q[head].next].key;
}

/**************************************************************************//**
*
* Return the key value for the given queue entry.
*
* \param  mInfo Pointer to MCOS structure.
* \param  tail The tail of the queue to grab the key from.
*
* \returns NULL or pointer to fcb_t.
*/

static __inline__ int32_t mcos_lastkey(mInfo_t * mInfo, uint16_t tail) {
	qentry_t	  * q = mInfo->q;

	isbadq(tail, E_ERROR);

	return q[q[tail].prev].key;
}

/**************************************************************************//**
*
* Return the first id value for the given queue entry.
*
* \param  mInfo Pointer to MCOS structure.
* \param  head The head index value to return the next queue entry.
*
* \returns E_ERROR or first id value.
*/

static __inline__ int16_t mcos_firstid(mInfo_t * mInfo, uint16_t head) {
	qentry_t  * q = mInfo->q;

	isbadq(head, E_ERROR);

	return q[head].next;
}

/**************************************************************************//**
*
* Return the fcb_t pointer to the pid.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid index balue to return the FCB pointer.
*
* \returns NULL or pointer to fcb_t.
*/

static __inline__ fcb_t * mcos_getfcb(mInfo_t * mInfo, uint16_t pid) {

	isbadpid(pid, NULL);

	return &mInfo->fcbs[pid];
}

/**************************************************************************//**
*
* Return the pid for the given fcb pointer.
*
* \param  mInfo Pointer to MCOS structure.
* \param  fcb The fcb pointer to grab the pid value from.
*
* \returns pid or E_ERROR
*/

static __inline__ uint16_t mcos_getpid(mInfo_t * mInfo, fcb_t * fcb) {

	if ( unlikely(fcb == NULL) )
		fcb = mInfo->curr;
	return (fcb->state == FCB_FREE) ? E_ERROR : fcb->pid;
}

/**************************************************************************//**
*
* Return the priority for the given fcb pointer.
*
* \param  fcb The fcb pointer value.
*
* \returns priority or E_ERROR
*/

static __inline__ uint16_t mcos_getprio(fcb_t * fcb) {
	return (unlikely(fcb->state == FCB_FREE)) ? E_ERROR : fcb->priority;
}

/**************************************************************************//**
*
* Execute the call to init_routine only once.
*
* \param  mInfo Pointer to MCOS structure.
* \param  once The structure pointer for testing if this routine has been
* called yet.
* \param  init_routine The routine to be called only once.
*
* \returns E_ERROR if error or E_OK if OK
*/

static __inline__ int32_t mcos_once(mInfo_t * mInfo, mcos_once_t * once, func_t init_routine) {
	if ( (once == NULL) || (init_routine == NULL) )
		return E_ERROR;

	if ( (*once != E_TRUE) && init_routine )
		init_routine(mInfo, 0);
	*once = E_TRUE;

	return E_OK;
}

/**************************************************************************//**
*
* Allocate and return a new pid value.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns E_ERROR on error or a new pid value
*/

static __inline__ int16_t __newpid(mInfo_t * mInfo)
{
	int16_t		i, pid;

	for (i = 0; likely(i < mInfo->num_fibers); i++) {
		if ( (pid = mInfo->nextfcb--) <= 0 )
			mInfo->nextfcb = (mInfo->num_fibers - 1);
		if ( mInfo->fcbs[pid].state == FCB_FREE )
			return pid;
	}
	return E_ERROR;
}

/**************************************************************************//**
*
* Free a given pid value.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid value for the FCB to be freed.
*
* \returns E_ERROR on error or a new pid value
*/

static __inline__ int32_t mcos_freepid(mInfo_t * mInfo, int16_t pid)
{
	fcb_t	  * fcb = &mInfo->fcbs[pid];

	isbadpid(pid, E_ERROR);

	dbgAssert( fcb->state != FCB_FREE );

	rte_free(fcb->uctx);
	rte_free(fcb->stack);

	fcb->state = FCB_FREE;

	return E_OK;
}

// Queue routines for FCB
/**************************************************************************//**
*
* Enqueue an item at the tail of a list.
*
* \param  mInfo Pointer to MCOS structure.
* \param  item The item to be enqueued to the tail of a list.
* \param  tail The index value of the tail of the queue.
*
* \returns item value.
*/

static __inline__ int16_t mcos_enqueue(mInfo_t * mInfo, int16_t item, int16_t tail) {
	qentry_t	  * tptr, * mptr;
	qentry_t	  * q = mInfo->q;

	dbgPrintf("item %d (%s) to tail %d\n", item, mInfo->fcbs[item].name, tail);
	tptr				= &q[tail];
	mptr				= &q[item];

	mptr->next			= tail;
	mptr->prev			= tptr->prev;
	q[tptr->prev].next	= item;
	tptr->prev			= item;

	return item;
}

/**************************************************************************//**
*
* Dequeue the item from a list and return the item.
*
* \param  mInfo Pointer to MCOS structure.
* \param  item The item to be dequeued from the tail of a list.
*
* \returns return the item
*/

static __inline__ int16_t mcos_dequeue(mInfo_t * mInfo, int16_t item) {
	qentry_t	  * mptr;
	qentry_t	  * q = mInfo->q;

	dbgPrintf("item %d (%s)\n", item, mInfo->fcbs[item].name);

	mptr	= &q[item];
	q[mptr->prev].next = mptr->next;
	q[mptr->next].prev = mptr->prev;

	return item;
}

/**************************************************************************//**
*
* Insert an item into a list or queue at a given location based on key value.
*
* \param  mInfo Pointer to MCOS structure.
* \param  proc The proc id or pid to be inserted into a list.
* \param  head The head index of the queue.
* \param  key The key value to determine where to insert the proc.
*
* \returns N/A
*/

static __inline__ void mcos_insert( mInfo_t * mInfo, uint16_t proc, uint16_t head, uint16_t key ) {
	qentry_t  * q = mInfo->q;
	int16_t		next, prev;

	isbadq(head, /* */);

	dbgPrintf("proc %d, head %d, key %d (%s)\n", proc, head, key, mcos_getfcb(mInfo, proc)->name);
	dbgPrintf("  next %d prev %d\n", q[proc].next, q[proc].prev);

	next = q[head].next;
	while (q[next].key < key) {	/* tail has max short as key	*/
		dbgPrintf("next %d key %d\n", next, q[next].key);
		next = q[next].next;
		dbgPrintf("next %d\n", next);
	}
	dbgPrintf("next %d prev %d\n", next, q[next].prev);
	q[proc].next	= next;
	q[proc].prev	= prev = q[next].prev;
	q[proc].key		= key;
	q[prev].next	= proc;
	q[next].prev	= proc;
	dbgLine( mcos_dbgListq(mInfo, "Insert", mInfo->rdyhead) );
}

/**************************************************************************//**
*
* Insert an item into a delta list or queue at a given location based on key value.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid value to be inserted into the head of a list.
* \param  head The head of the list to be inserted into.
* \param  key The key value to be used to sort the pid into the list.
*
* \returns N/A
*/

static __inline__ void mcos_insertd( mInfo_t * mInfo, uint16_t pid, uint16_t head, int32_t key ) {

	int16_t	next, prev;
	qentry_t  * q;

	isbadq(head, /* */);

	dbgAssert( mInfo != NULL );
	q = mInfo->q;

	dbgPrintf("pid %d, head %d, key %d\n", pid, head, key);

	for(prev = head, next = q[head].next;
			q[next].key < key;
			prev = next, next = q[next].next)
		key -= q[next].key;

	q[pid].next		= next;
	q[pid].prev		= prev;
	q[pid].key		= key;

	q[prev].next	= pid;
	q[next].prev	= pid;

	if ( next < mInfo->num_fibers )
		q[next].key -= key;
}

/**************************************************************************//**
*
* Return true if the list is empty.
*
* \param  mInfo Pointer to MCOS structure.
* \param  qid The queue index value to test.
*
* \returns return boolean true if list is empty or false if not
*/

static __inline__ int32_t mcos_isempty(mInfo_t * mInfo, uint16_t qid) {
	qentry_t * q = (qentry_t *)&mInfo->q[qid];
	return (q->next == q->prev);
}

/**************************************************************************//**
*
* Return the a boolean true value if the queue is non-empty.
*
* \param  mInfo Pointer to MCOS structure.
* \param  qid The queue index value to test.
*
* \returns boolean value true if not empty or false
*/

static __inline__ int32_t mcos_nonempty( mInfo_t * mInfo, uint16_t qid ) {
	qentry_t * q = (qentry_t *)&mInfo->q[qid];
	return (q->next != q->prev);
}

/**************************************************************************//**
*
* Delay the fiber by the number of clock ticks.
*
* \param  mInfo Pointer to MCOS structure.
* \param  ticks The number of ticks to delay or sleep.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_delay(mInfo_t * mInfo, uint32_t ticks) {
	qentry_t	  * q = &mInfo->q[0];

	dbgAssert( mInfo != NULL );
	if ( unlikely((mInfo->flags & CLOCK_FLAG) == 0) )
		return E_ERROR;

	if ( likely(ticks) ) {
		// Add the current pid to the priority timer queue.
		mcos_insertd(mInfo, mInfo->currpid, mInfo->sleepq, ticks);

		mInfo->curr->state	= FCB_SLEEP;
		mInfo->sltop		= &q[q[mInfo->sleepq].next];
		mInfo->slnempty		= E_TRUE;
		mInfo->curr->semid	= mInfo->sleepq;

		dbgLine( mcos_dbgListq(mInfo, "sleepq", mInfo->sleepq) );
	}
	mcos_resched(mInfo);
	return E_OK;
}

/**************************************************************************//**
*
* Delay the fiber by the number of seconds
*
* \param  mInfo Pointer to MCOS structure.
* \param  sec The number of seconds to sleep or delay.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_sleep(mInfo_t * mInfo, uint32_t sec) {

	dbgAssert( mInfo != NULL );
	if ( (mInfo->flags & CLOCK_FLAG) == 0 )
		return E_ERROR;

	dbgPrintf("seconds %d\n", sec);
	if ( sec ) {
		while ( sec >= 1000 ) {
			mcos_delay(mInfo, mInfo->tickrate * 1000);
			sec -= 1000;
		}
		if ( sec > 0 )
			mcos_delay(mInfo, mInfo->tickrate * sec);
	}
	return E_OK;
}

/**************************************************************************//**
*
* Delay the fiber by the number of milliseconds.
*
* \param  mInfo Pointer to MCOS structure.
* \param  ms The number of milli-seconds to delay or sleep.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_delay_ms(mInfo_t * mInfo, uint32_t ms) {
	uint32_t		ticks;

	dbgAssert( mInfo != NULL );
	if ( (mInfo->flags & CLOCK_FLAG) == 0 )
		return E_ERROR;

	dbgPrintf("milliseconds %d\n", ms);
	ticks = ms/(1000/mInfo->tickrate);
	if ( ticks == 0 )
		ticks = 1;

	return mcos_delay(mInfo, ticks);
}

/**************************************************************************//**
*
* Create or allocate a new queue entry.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns queue id value or E_ERROR
*/

static __inline__ uint16_t __newq(mInfo_t * mInfo) {
	qentry_t  * hptr, * tptr;
	uint16_t	hindex, tindex;
	qentry_t  * q = mInfo->q;

	hptr = &q[hindex = mInfo->nextqueue++];
	tptr = &q[tindex = mInfo->nextqueue++];

	hptr->next	= tindex;
	hptr->prev	= Q_EMPTY;
	hptr->key	= MINSHORT;

	tptr->next	= Q_EMPTY;
	tptr->prev	= hindex;
	tptr->key	= MAXSHORT;

	return hindex;
}

/**************************************************************************//**
*
* Remove the first fcb_t pointer from a given list head.
*
* \param  mInfo Pointer to MCOS structure.
* \param  head The head index value of the queue.
*
* \returns NULL if error or fcb_t pointer to first item
*/

static __inline__ int16_t mcos_getfirst( mInfo_t * mInfo, int16_t head ) {
	int16_t		item;
	qentry_t  * q = mInfo->q;

	isbadq(head, E_EMPTY);

	dbgPrintf("head %d next %d num_fibers %d\n", head, q[head].next, mInfo->num_fibers);

	if ( likely((item = q[head].next) < mInfo->num_fibers) )
		return mcos_dequeue(mInfo, item);
	return E_EMPTY;
}

/**************************************************************************//**
*
* Get the last fcb_t pointer in a given list.
*
* \param  mInfo Pointer to MCOS structure.
* \param  tail The tail of the queue.
*
* \returns EMPTY or pid
*/

static __inline__ int16_t mcos_getlast( mInfo_t * mInfo, int16_t tail ) {
	int16_t		item;
	qentry_t  * q = mInfo->q;

	isbadq(tail, E_EMPTY);

	dbgPrintf("tail %d prev %d num_fibers %d\n", tail, q[tail].prev, mInfo->num_fibers);

	if ( likely((item = q[tail].prev) < mInfo->num_fibers) )
		return mcos_dequeue(mInfo, item);
	return E_EMPTY;
}

/****************************************************************************/

/**************************************************************************//**
*
* Create a new semaphore entry.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns queue id value or E_ERROR
*/

static __inline__ uint16_t __newsem(mInfo_t * mInfo)
{
	int16_t		i, sem;

	for (i = 0; likely(i < mInfo->num_semas); i++) {
		if ( (sem = mInfo->nextsema--) < 0 )
			mInfo->nextsema = (mInfo->num_semas - 1);
		if ( mInfo->sem[sem].state == S_FREE ) {
			mInfo->sem[sem].state = S_USED;
			return sem;
		}
	}
	return E_ERROR;
}

/**************************************************************************//**
*
* Initialize a new semaphore and initialize the data structure.
*
* \param  mInfo Pointer to MCOS structure.
* \param  count The number initial count value of the semaphore.
*
* \returns N/A
*/

static __inline__ int16_t mcos_screate(mInfo_t * mInfo, int16_t count) {
	int16_t		sem;

	if ( unlikely((sem = __newsem(mInfo)) == E_ERROR) )
		return E_ERROR;

	mInfo->sem[sem].count	= count;
	rte_wmb();

	return sem;
}

/**************************************************************************//**
*
* Create a new mutex semaphore entry.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns mutex id value or E_ERROR
*/

static __inline__ uint16_t __newmutex(mInfo_t * mInfo)
{
	int16_t		i, m;

	for (i = 0; likely(i < mInfo->num_mutexs); i++) {
		if ( (m = mInfo->nextmutex--) < 0 )
			mInfo->nextmutex = (mInfo->num_mutexs - 1);
		if ( mInfo->mutex[m].owner == M_FREE ) {
			mInfo->mutex[m].owner = M_EMPTY;
			return m;
		}
	}
	return E_ERROR;
}

/**************************************************************************//**
*
* Initialize a new mutext semaphore and initialize the data structure.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns N/A
*/

static __inline__ int16_t mcos_mcreate(mInfo_t * mInfo) {
	int16_t		m;

	if ( unlikely((m = __newmutex(mInfo)) == E_ERROR) )
		return E_ERROR;

	mInfo->mutex[m].recurse	= 0;
	rte_wmb();

	return m;
}

/**************************************************************************//**
*
* Set the given fcb_t to the ready state and put on ready queue.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid index value to use.
* \param  resched The flag to allow a reschedule call to happen.
*
* \returns N/A
*/

static __inline__ void mcos_ready(mInfo_t * mInfo, uint16_t pid, int16_t resched) {
	fcb_t	  * pFcb = &mInfo->fcbs[pid];

	isbadpid(pid, /* */);
	dbgAssert( mInfo != NULL );

	dbgPrintf("pid %d (%s) %s\n", pid, mcos_getfcb(mInfo, pid)->name,
			(resched == 0) ? "Do not resched" : "resched");
	pFcb->state = FCB_READY;
	pFcb->semid	= 0;
	mcos_insert(mInfo, pFcb->pid, mInfo->rdyhead, pFcb->priority);
	if ( resched ) {
		dbgLine( mcos_dbgListq(mInfo, "Ready 0", mInfo->rdyhead) );
		mcos_resched(mInfo);
	}
}

/**************************************************************************//**
*
* Delete the given semaphore id value and free the structure.
*
* \param  mInfo Pointer to MCOS structure.
* \param  sem The semaphore index value.
*
* \returns old semaphore id value or E_ERROR if bad
*/

static __inline__ int32_t mcos_sdelete(mInfo_t * mInfo, uint16_t sem) {
	int16_t		pid;

	if ( sem == (uint16_t)E_ERROR )
		return E_OK;

	isbadsem(sem, E_ERROR);

	dbgAssert( mInfo->sem[sem].state != S_FREE );

	mInfo->sem[sem].state	= S_FREE;
	if ( mcos_nonempty(mInfo, sem) ) {
		while( (pid = mcos_getfirst(mInfo, mInfo->sem[sem].head)) != E_EMPTY)
			mcos_ready(mInfo, pid, 0);
		mcos_resched(mInfo);
	}

	return E_OK;
}

/**************************************************************************//**
*
* Delete the given mutex semaphore id value and free the structure.
*
* \param  mInfo Pointer to MCOS structure.
* \param  m The mutex index value.
*
* \returns old semaphore id value or E_ERROR if bad
*/

static __inline__ int32_t mcos_mdelete(mInfo_t * mInfo, uint16_t m) {
	int16_t		pid;

	if ( m == (uint16_t)E_ERROR )
		return E_OK;

	dbgAssert( mInfo->mutex[m].owner != M_FREE );

	mInfo->mutex[m].owner	= M_FREE;
	if ( mcos_nonempty(mInfo, m) ) {
		while( (pid = mcos_getfirst(mInfo, mInfo->mutex[m].head)) != E_EMPTY)
			mcos_ready(mInfo, pid, 0);
		mcos_resched(mInfo);
	}

	return E_OK;
}

/**************************************************************************//**
*
* Suspend a given fiber.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid value to be used to suspend.
*
* \returns priority value
*/

static __inline__ int32_t mcos_suspend(mInfo_t * mInfo, uint16_t pid) {
	fcb_t	  * fcb;

	isbadpid(pid, E_ERROR);

	if ( (fcb = &mInfo->fcbs[pid])->state == FCB_READY ) {
		mcos_dequeue(mInfo, pid);
		fcb->state = FCB_SUSPEND;
	} else {
		fcb->state = FCB_SUSPEND;
		mcos_resched(mInfo);
	}

	return E_OK;
}

/**************************************************************************//**
*
* Resume a suspended fiber and return the priority.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid index value.
*
* \returns Priority value or E_ERROR
*/

static __inline__ uint16_t mcos_resume(mInfo_t * mInfo, uint16_t pid) {
	fcb_t	  * fcb;
	uint16_t	prio;

	isbadpid(pid, E_ERROR);

	dbgPrintf("pid %d(%s)\n", pid, mcos_getfcb(mInfo, pid)->name);
	if ( unlikely((fcb = &mInfo->fcbs[pid])->state != FCB_SUSPEND) )
		return E_ERROR;

	prio = fcb->priority;
	mcos_ready(mInfo, pid, RESCHED_YES);
	return prio;
}

/**************************************************************************//**
*
* Return the number of waiting fibers. We assume sem is valid.
*
* \param  mInfo Pointer to MCOS structure.
* \param  semid The semaphore index value.
*
* \returns E_ERROR or Number of waiting fibers.
*/

static __inline__ int32_t mcos_semcnt(mInfo_t * mInfo, int16_t semid) {

	isbadsem(semid, E_SEM_INVALID);

	dbgAssert( mInfo->sem[semid].state != S_FREE );

	return mInfo->sem[semid].count;
}

/**************************************************************************//**
*
* Wait on a given semaphore.
*
* \param  mInfo Pointer to MCOS structure.
* \param  semid The semaphore index value.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_wait(mInfo_t * mInfo, int16_t semid) {

	sentry_t  * s;

	isbadsem(semid, E_SEM_INVALID);

	s = &mInfo->sem[semid];
	dbgAssert( s->state != S_FREE );

	dbgPrintf("Take semaphore %d count %d (%s) pid %d\n",
			semid, s->count, mInfo->curr->name, mInfo->currpid);
	if ( --(s->count) < 0 ) {
		mInfo->curr->state	= FCB_WAITING;
		mInfo->curr->semid	= semid;
		mcos_enqueue(mInfo, mInfo->currpid, s->tail);
		dbgLine( mcos_dbgLists(mInfo, "Sema", semid) );
		mcos_resched(mInfo);
	}
	return E_OK;
}

/**************************************************************************//**
*
* Try to take a given semaphore.
*
* \param  mInfo Pointer to MCOS structure.
* \param  semid The semaphore index value.
*
* \returns Zero on success or non-zero on failure
*/

static __inline__ int32_t mcos_try(mInfo_t * mInfo, int16_t semid) {

	sentry_t  * s;

	isbadsem(semid, E_SEM_INVALID);

	s = &mInfo->sem[semid];
	dbgAssert( s->state != S_FREE );

	dbgPrintf("Try semaphore %d count %d (%s) pid %d\n",
			semid, s->count, mInfo->curr->name, mInfo->currpid);
	--(s->count);
	return (s->count >= 0)? 0 : 1;
}

/**************************************************************************//**
*
* Take a given mutex semaphore and wait if required.
*
* \param  mInfo Pointer to MCOS structure.
* \param  m The mutex index value.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_take(mInfo_t * mInfo, int16_t m) {

	mentry_t	* mx = &mInfo->mutex[m];

	dbgPrintf("m %d\n", m);
	dbgAssert( mInfo->mutex[m].owner != M_FREE );

	if ( likely(mx->owner == M_EMPTY) ) {
		mx->owner = mInfo->currpid;
		dbgPrintf("Take ownership %d (%s)\n", mx->owner, mInfo->curr->name);
		return E_OK;
	}

	dbgPrintf("owner %d recurse %d\n", mx->owner, mx->recurse);
	if ( likely(mx->owner == mInfo->currpid) ) {
		mx->recurse++;
		rte_wmb();
		return E_OK;
	}

	dbgPrintf("Put on queue to wait\n");
	mInfo->curr->state	= FCB_MUTEX_WAIT;
	mInfo->curr->semid	= m;
	mcos_enqueue(mInfo, mInfo->currpid, mx->tail);
	dbgLine( mcos_dbgLists(mInfo, "mutex", m) );
	mcos_resched(mInfo);

	return E_OK;
}

/**************************************************************************//**
*
* signal a given mutex semaphore.
*
* \param  mInfo Pointer to MCOS structure.
* \param  m The mutex index value.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_give(mInfo_t * mInfo, int16_t m) {

	mentry_t  * mx;
	int16_t		pid;

	mx = &mInfo->mutex[m];
	dbgAssert( mx->owner != M_FREE );

	dbgPrintf("m %d owner %d (%s) recurse %d\n",
			m, mx->owner, mInfo->curr->name, mx->recurse);

	if ( unlikely(mx->owner != mInfo->currpid) )
		return E_ERROR;

	if ( mx->recurse == 0 ) {
		mx->owner = M_EMPTY;
		return E_OK;
	}

	mx->recurse--;
	rte_wmb();

	dbgPrintf("Wakeup the first waiting task\n");
	mx->owner = M_EMPTY;
	pid = mcos_getfirst(mInfo, mx->head);
	dbgPrintf("pid %d\n", pid);
	if ( pid != E_ERROR )
		mcos_ready(mInfo, pid, 1);

	return E_OK;
}

/**************************************************************************//**
*
* signal a given semaphore.
*
* \param  mInfo Pointer to MCOS structure.
* \param  semid The semaphore index value.
*
* \returns E_ERROR or E_OK
*/

static __inline__ int32_t mcos_signal(mInfo_t * mInfo, int16_t semid) {

	sentry_t  * s;

	isbadsem(semid, E_ERROR);

	s = &mInfo->sem[semid];
	dbgAssert( s->state != S_FREE );

	dbgPrintf("sem %d count %d (%s)\n", semid, s->count, mInfo->curr->name);
	if ( (s->count++) < 0 )
		mcos_ready(mInfo, mcos_getfirst(mInfo, s->head), 1);

	return E_OK;
}

/**************************************************************************//**
*
* Flush the Semaphore of all waiting fibers.
*
* \param  mInfo Pointer to MCOS structure.
* \param  sem The semaphore index value.
*
* \returns Number of fibers flushed.
*/

static __inline__ int16_t mcos_semflush(mInfo_t * mInfo, int16_t sem) {

	int16_t		semcnt = mcos_semcnt(mInfo, sem);

	dbgPrintf("semcnt %d\n", semcnt);
	while( mcos_semcnt(mInfo, sem) != 0 )
		mcos_signal(mInfo, sem);

	return semcnt;
}

/**************************************************************************//**
*
* Wake up all waiting fibers on the sleepq.
*
* \param  mInfo Pointer to MCOS structure.
* \param flag The reschedule flag RESCHED_YES or RESCHED_NO.
*
* \returns Number of fibers flushed.
*/

static __inline__ int32_t mcos_wakeup(mInfo_t * mInfo, int32_t flag ) {

	qentry_t  * q = mInfo->q;
	int16_t		cnt = 0;

	// Wake up everyone that is ready to run on the clock queue.
	while( mcos_nonempty(mInfo, mInfo->sleepq) && (mcos_firstkey(mInfo, mInfo->sleepq) == 0) ) {
		mcos_ready(mInfo, mcos_getfirst(mInfo, mInfo->sleepq), 0);
		cnt++;
	}

	if ( (mInfo->slnempty = mcos_nonempty(mInfo, mInfo->sleepq)) )
		mInfo->sltop = &q[q[mInfo->sleepq].next];

	if ( flag )
		mcos_resched(mInfo);

	return cnt;
}

/**************************************************************************//**
*
* Stop the clock for a while.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns N/A
*/

static __inline__ void mcos_clkstop(mInfo_t * mInfo)
{
	mInfo->defclk++;
}

/**************************************************************************//**
*
* Kill a given fiber id.
*
* \param  mInfo Pointer to MCOS structure.
* \param  pid The pid value used to kill.
*
* \returns E_OK or E_ERROR
*/

static __inline__ int32_t mcos_kill(mInfo_t * mInfo, uint16_t pid)
{
	fcb_t	  * fcb;

	isbadpid(pid, E_ERROR);

	dbgPrintf("pid %d\n", pid);
	if ( unlikely((fcb = &mInfo->fcbs[pid])->state == FCB_FREE) )
		return E_ERROR;

	switch( fcb->state ) {
	case FCB_CURR:	mcos_freepid(mInfo, fcb->pid);
					mcos_resched(mInfo);
					break;
	case FCB_WAITING:	mInfo->sem[fcb->semid].count++;
					mcos_dequeue(mInfo, fcb->pid);
					mcos_freepid(mInfo, fcb->pid);
					break;
	case FCB_SLEEP:
	case FCB_READY:	mcos_dequeue(mInfo, fcb->pid);
					mcos_freepid(mInfo, fcb->pid);
					break;
	default:		mcos_freepid(mInfo, fcb->pid);
					break;
	}

	return E_OK;
}

/**************************************************************************//**
*
* Stop the system.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns N/A
*/

static __inline__ void mcos_stop(mInfo_t * mInfo) {
	fcb_t	  * fcb;
	int32_t		i;

	if ( likely(mInfo != NULL) ) {
		for(i = 0, fcb = mInfo->fcbs; fcb && (i < mInfo->num_fibers); i++, fcb++)
			mcos_kill(mInfo, fcb->pid);

		mInfo->flags |= STOP_FLAG;
		rte_wmb();
	}
}

/**************************************************************************//**
*
* Change the priority for the given fcb pointer.
*
* \param  mInfo Pointer to MCOS structure.
* \param  fcb The FCB pointer to the fiber to change.
* \param  prio The priority to be change.
*
* \returns old priority or E_ERROR
*/

static __inline__ uint16_t mcos_chgprio(mInfo_t * mInfo, fcb_t * fcb, uint16_t prio) {
	uint16_t	old = fcb->priority;

	if ( unlikely(fcb->state == FCB_FREE) )
		return E_ERROR;

	fcb->priority = prio;
	switch( fcb->state ) {
	case FCB_READY:		mcos_dequeue(mInfo, fcb->pid);
						mcos_insert(mInfo, fcb->pid, fcb->semid, fcb->pid);
						mcos_resched(mInfo);
						break;
	case FCB_CURR:		mcos_resched(mInfo);
						break;
	default:			break;
	}
	return old;
}

/**************************************************************************//**
*
* Clean up the memory allocation and free the mInfo_t structure.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns NULL if error or a void pointer to fcb_t structure.
*/

static __inline__ void mcos_cleanup(mInfo_t * mInfo)
{
	if ( unlikely(mInfo == NULL) )
		return;

	rte_free(mInfo->q);
	rte_free(mInfo->fcbs);
	rte_free(mInfo->vector);
	rte_free(mInfo->done->uc_stack.ss_sp);
	rte_free(mInfo->done);
	rte_free(mInfo->fcbs);
	rte_free(mInfo->mutex);
	rte_free(mInfo->sem);
	rte_free(mInfo);
}

/**************************************************************************//**
*
* Set a pid into a given vector and forget any value before.
*
* \param  mInfo Pointer to MCOS structure.
* \param  v The vector number to be set.
* \param  pid The pid to be used for the vector.
*
* \returns N/A
*/

static __inline__ void mcos_set_vector(mInfo_t * mInfo, int16_t v, int16_t pid) {

	isbadpid(pid, /* */);

	mInfo->vector[v] = pid;
	dbgPrintf("Set Vector %d for (%s)\n", v,
			(mInfo->vector[v] != FCB_EMPTY) ? mcos_getfcb(mInfo, mInfo->vector[v])->name : "Empty");
}

/**************************************************************************//**
*
* Clear the interrupts in the bitmap for the given vector.
*
* \param  mInfo Pointer to MCOS structure.
*
* \returns The original interrupt bits
*/

static __inline__ int32_t mcos_clear_interrupts(mInfo_t * mInfo) {

	uint32_t	ret = rte_atomic32_read(&mInfo->vector_bitmap);
	uint32_t	bitmap = ret;

	while( rte_atomic32_cmpset((uint32_t *)&mInfo->vector_bitmap, bitmap, 0) == 0 )
		bitmap = rte_atomic32_read(&mInfo->vector_bitmap);

	return ret;
}

/**************************************************************************//**
*
* Get the current tick rate as a 64 bit value.
*
* \returns The current tick rate as a 64 bit value
*/

static __inline__ uint64_t mcos_get_tick64(mInfo_t * mInfo) {
	return mInfo->clk;
}

/**************************************************************************//**
*
* Get the current tick rate as a 32 bit value.
*
* \returns The current tick rate as a 32 bit value
*/

static __inline__ uint32_t mcos_get_tick32(mInfo_t * mInfo) {
	return (uint32_t)(mcos_get_tick64(mInfo) & 0xFFFFFFFFL);
}

/**************************************************************************//**
*
* Get the current tick rate as a 64 bit value.
*
* \returns The current tick rate as a 64 bit value
*/

static __inline__ uint64_t mcos_get_timestamp(__attribute__ ((unused))mInfo_t * mInfo) {
	return rte_get_timer_hz();
}

/****************************************************************************/

#endif /* rte_mcos_H_ */

#include "rte_mcos_lifo.h"
#include "rte_mcos_fifo.h"
#include "rte_mcos_debug.h"

#ifdef RTE_ENABLE_VIPER_API
#include "rte_viper.h"
#endif

