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
 * Viper header file with many inline functions and structures.
 *
 * Created by Keith Wiles @ Wind River Systems (2014)
 *
 */

#ifndef rte_viper_H_
#define rte_viper_H_				/**< Used to not have recursive includes */

#ifndef NULL
#define NULL	0
#endif

#define	VIPER_FIFO_SIZE		128
#define VIPER_LIFO_SIZE		128
#define VIPER_STACK_SIZE	128

typedef struct viper_s {
	mInfo_t		  * mInfo;
	uint16_t		lifo_size;
	uint16_t		fifo_size;
	uint16_t		stack_size;
} viper_t;

extern __thread viper_t	  * __viper;

typedef unsigned int VIRT_ADDR;
typedef unsigned int PHYS_ADDR;

typedef struct {
	int16_t		sem;
} NANO_SEM;

typedef struct {
	lifo_t	  * lifo;
	uint16_t	size;
} NANO_LIFO;

typedef struct {
	fifo_t	  * fifo;
	uint16_t	size;
} NANO_FIFO;

typedef struct {
	uint32_t	active;
	uint32_t	timeout;
	void	  * userdata;
} NANO_TIMER;

typedef struct {
	void	  * ptr;
} NANO_CTX;

typedef NANO_CTX * NANO_CTX_ID;

typedef struct {
	void	  * ptr;
} NANO_VM_CTX;

typedef NANO_VM_CTX	  * NANO_VM_CTX_ID;

typedef struct {
	lifo_t	  * stack;
	uint16_t	size;
} NANO_STACK;

typedef void (*nanoFiberEntry) (int i1, int i2);

typedef enum { NANO_CTX_ISR = 1, NANO_CTX_TASK, NANO_CTX_FIBER } NANO_CTX_TYPE;

/****************************************************************************/
/*                       NanoKernel Only APIs                               */
/****************************************************************************/

/****************************************************************************/

static __inline__ void * nanoFiberTimerGet(NANO_TIMER * timer) {

	return (timer->timeout == 0)? timer->userdata : NULL;
}

static __inline__ void * nanoFiberTimerGetW(NANO_TIMER * timer) {

	void	  * userdata;

	// Wait for the timer to expire
	while( (userdata = nanoFiberTimerGet(timer)) == NULL )
		mcos_yield(__viper->mInfo);

	return userdata;
}

static __inline__ void nanoFiberTimerStart(NANO_TIMER * timer, int ticks) {

	timer->timeout	= ticks;
	timer->active	= 1;
}

static __inline__ void nanoFiberTimerStop(NANO_TIMER * timer) {

	timer->active	= 0;
}

/****************************************************************************/

static __inline__ void * nanoTaskTimerGet(NANO_TIMER * timer) {

	return nanoFiberTimerGet(timer);
}

static __inline__ void * nanoTaskTimerGetW(NANO_TIMER * timer) {

	return nanoFiberTimerGetW(timer);
}

static __inline__ void nanoTaskTimerStart(NANO_TIMER * timer, int ticks) {

	nanoFiberTimerStart(timer, ticks);
}

static __inline__ void nanoTaskTimerStop(NANO_TIMER * timer) {

	nanoFiberTimerStop(timer);
}

/****************************************************************************/

static __inline__ uint32_t nanoTimeElapseGet(void) {
	uint64_t	elapsed = __viper->mInfo->elapsedTime;

	__viper->mInfo->elapsedTime = mcos_get_tick64(__viper->mInfo);

	return (uint32_t)(__viper->mInfo->elapsedTime - elapsed);
}

static __inline__ void nanoTimeInit(int prio) {
	/*
	 * Should be started already from MCOS.
	 * mcos_clkinit(__viper->mInfo);
	 * mcos_clkStart(__viper->mInfo);
	*/
}

static __inline__ uint32_t nanoTimeStampGet(void) {
	return (uint32_t)mcos_get_timestamp(__viper->mInfo);
}

static __inline__ uint32_t nanoTimeTickGet(void) {
	return mcos_get_tick32(__viper->mInfo);
}

/****************************************************************************/

static __inline__ void nanoTimerInit(NANO_TIMER * timer, void * userdata) {
	timer->active	= 0;
	timer->timeout	= 0;
	timer->userdata	= userdata;
}

/****************************************************************************/
/*                       NanoKernel APIs                                    */
/****************************************************************************/

/****************************************************************************/

static __inline__ void * nanoAppHandleGet(void) {

	return __viper->mInfo->curr->priv;
}

static __inline__ void nanoAppHandleSet(void * value) {

	__viper->mInfo->curr->priv = value;
}

/****************************************************************************/

static __inline__ void nanoCpuFiberFpDisable(NANO_CTX_ID ctx) {

}

static __inline__ void nanoCpuFiberFpEnable(NANO_CTX_ID ctx, int options) {

}

/****************************************************************************/

static __inline__ void nanoCpuIntConnect( int interrupt_number,
						void (*handler_entry)(int arg), ...) {

}

static __inline__ void nanoCpuIntDisable(int irq) {

}

static __inline__ void nanoCpuIntEnable(int irq) {

}

static __inline__ void nanoCpuIntHandleSet(int vector,
						void (*oldRoutine)(void *param),
						void (*newRoutine)(void *param),
						void * param) {

}

static __inline__ unsigned int nanoCpuIntLock(void) {

	return 0;
}

static __inline__ void nanoCpuIntUnlock(unsigned int key) {

}

/****************************************************************************/

static __inline__ VIRT_ADDR nanoCpuMmioBufAlloc(unsigned int numBytes) {

	return 0;
}

static __inline__ VIRT_ADDR nanoCpuMmioMap(VIRT_ADDR virt, PHYS_ADDR phys,
								unsigned int size, int cache ) {

	return 0;
}

static __inline__ PHYS_ADDR nanoCpuMmuVirtToPhys(NANO_VM_CTX_ID vmCtxId, VIRT_ADDR virt) {

	return 0;
}

/****************************************************************************/

static __inline__ void nanoCpuTaskFpDisable(NANO_CTX_ID ctx) {

	nanoCpuFiberFpDisplay(ctx);
}

static __inline__ void nanoCpuTaskFpEnable(NANO_CTX_ID ctx, unsigned int options) {

	nanoCpuFiberFpEnable(ctx, options);
}

/****************************************************************************/

static __inline__ NANO_CTX_ID nanoCtxSelf(void) {

	return (NANO_CTX_ID)mcos_getfcb(__viper->mInfo, mcos_getpid(__viper->mInfo, NULL));
}

static __inline__ NANO_CTX_TYPE nanoCtxType(void) {

	return NANO_CTX_FIBER;
}

/****************************************************************************/

static __inline__ void * nanoFiberFifoGet(NANO_FIFO * chan) {
	data_u	  * data;

	if ( mcos_fpop(chan->fifo, &data) == E_OK )
		return data->pdata;
	else
		return NULL;
}

static __inline__ void * nanoFiberFifoGetW(NANO_FIFO * chan) {
	data_u	  * data;

	if ( mcos_fpop_wait(chan->fifo, &data) == E_OK )
		return data->pdata;
	else
		return NULL;
}

static __inline__ void nanoFiberFifoPut(NANO_FIFO * chan, void * data) {
	mcos_fpush(chan->fifo, (data_u *)data);
}

/****************************************************************************/

static __inline__ void * nanoFiberLifoGet(NANO_LIFO * chan) {
	data_u	  * data;

	if ( mcos_lpop(chan->lifo, &data) == E_OK )
		return data->pdata;
	else
		return NULL;
}

static __inline__ void * nanoFiberLifoGetW(NANO_LIFO * chan) {
	data_u	  * data;

	if ( mcos_lpop_wait(chan->lifo, &data) == E_OK )
		return data->pdata;
	else
		return NULL;
}

static __inline__ void nanoFiberLifoPut(NANO_LIFO * chan, void * data) {

	mcos_lpush(chan->lifo, data);
}

/****************************************************************************/

static __inline__ void nanoFiberSemGive(NANO_SEM * chan) {

	mcos_signal(__viper->mInfo, chan->sem);
}

static __inline__ void nanoFiberSemTake(NANO_SEM * chan) {

	mcos_wait(__viper->mInfo, chan->sem);
}

static __inline__ void nanoFiberSemTakeW(NANO_SEM * chan) {

	mcos_try(__viper->mInfo, chan->sem);
}

/****************************************************************************/

static __inline__ int nanoFiberStackPop(NANO_STACK * chan, uint32_t * pData) {

	return nanoFiberLifoGet((NANO_LIFO *)chan, pData);
}

static __inline__ uint32_t nanoFiberStackPopW(NANO_STACK * chan) {

	uint32_t	  * data;

	data = nanoFiberLifoGetW((NANO_LIFO *)chan);

	return *data;
}

static __inline__ void nanoFiberStackPush(NANO_STACK * chan, uint32_t data) {

	nanoFiberLifoPut((NANO_LIFO *)chan, (void *)((long long)data));
}

/****************************************************************************/

/**************************************************************************//**
*
* Create a fiber routine.
*
* \param  stack Pointer to a stack memory area.
* \param  size Amount of bytes for the stack if zero then use default value.
* \param  entry Routine to call to start the fiber. Prototype for the routine
* is `void * entry(void *, int arg);`
* \param  prio Priority value for fiber higer the number the high the priority.
* \param  arg1 Argument 1
* \param  arg2 Argument 2 (not used)
* \param  options Argument for the fiber (Not used)
*
* \returns N/A
*/

static __inline__ void nanoFiberStart(char * stack, unsigned size,
		void (*entry)(void *, int),	int arg1, int arg2, unsigned prio,
		unsigned options) {
	fcb_t	  * pFcb;

	dbgPrintf("Entry (%p, %d)\n", stack, size);

	(void)__spawn(__viper->mInfo, "Fiber", prio, stack, size, entry, arg1, &pFcb);

	pFcb->priv = (void *)((long long)arg2);

	dbgPrintf("Leave (%p, %d)\n", stack, size);
}

/****************************************************************************/

static __inline__ void nanoFiberYeild(void) {
	mcos_yeild(__viper->mInfo);
}

static __inline__ void nanoFifoSizeSet(uint32_t size) {

	__viper->fifo_size = size;
}

static __inline__ void nanoFifoInit(NANO_FIFO * chan) {

	nanoFifoSizeSet(chan->size);
	if ( __viper->fifo_size == 0 )
		nanoFifoSizeSet(VIPER_FIFO_SIZE);
	chan->fifo = mcos_fcreate(__viper->mInfo, chan->fifo->data, __viper->fifo_size);
}

/****************************************************************************/

static __inline__ void * nanoIsrFifoGet(NANO_FIFO * chan) {

	return nanoFiberFifoGet(chan);
}

static __inline__ void nanoIsrFifoPut(NANO_FIFO * chan, void * data) {

	nanoFiberFifoPut(chan, data);
}

static __inline__ void * nanoIsrLifoGet(NANO_LIFO * chan) {

	return nanoFiberLifoGet(chan);
}

static __inline__ void nanoIsrLifoPut(NANO_LIFO * chan, void * data) {

	nanoFiberLifoPut(chan, data);
}

static __inline__ void nanoIsrSemGive(NANO_SEM * chan) {

	nanoFiberSemGive(chan);
}

static __inline__ void nanoIsrSemTake(NANO_SEM * chan) {

	nanoFiberSemTake(chan);
}

static __inline__ int nanoIsrStackPop(NANO_STACK * chan, uint32_t * pData) {

	return nanoFiberStackPop(chan, pData);
}

static __inline__ void nanoIsrStackPush(NANO_STACK * chan, uint32_t data) {

	nanoFiberStackPush(chan, data);
}

/****************************************************************************/

static __inline__ void nanoLifoSizeSet(uint32_t size) {

	__viper->lifo_size = size;
}

static __inline__ void nanoLifoInit(NANO_LIFO * chan) {

	nanoLifoSizeSet(chan->size);
	if ( __viper->lifo_size == 0 )
		nanoLifoSizeSet(VIPER_LIFO_SIZE);
	chan->lifo = mcos_lcreate(__viper->mInfo, NULL, __viper->lifo_size);
}

/****************************************************************************/

static __inline__ void nanoPhysPgPoolAdd(PHYS_ADDR pool, unsigned int size) {

}

static __inline__ unsigned int nanoPhysPgPoolSizeGet(void) {

	return 0;
}

/****************************************************************************/

static __inline__ void nanoSemInit(NANO_SEM * chan) {

	chan->sem = mcos_screate(__viper->mInfo, 0);
}

static __inline__ void nanoStackSizeSet(uint32_t size) {

	__viper->stack_size = size;
}

static __inline__ void nanoStackInit(NANO_STACK * chan, uint32_t * pData) {

	chan->stack->data	= (data_u *)pData;
	chan->stack->count	= 0;
	nanoStackSizeSet(chan->size);
	if ( __viper->stack_size == 0 )
		nanoStackSizeSet(VIPER_STACK_SIZE);
	chan->stack->size		= __viper->stack_size;
}

/****************************************************************************/

static __inline__ void nanoTaskFiberStart(char * stack, unsigned stackSize,
						nanoFiberEntry pEntry,
						int parameter1, int parameter2,
						unsigned priority, unsigned options) {

}

/****************************************************************************/

static __inline__ void * nanoTaskFifoGet(NANO_FIFO * chan) {

	return nanoFiberFifoGet(chan);
}

static __inline__ void * nanoTaskFifoGetW(NANO_FIFO * chan) {

	return nanoFiberFifoGetW(chan);
}

static __inline__ void nanoTaskFifoPut(NANO_FIFO * chan, void * data) {

	nanoFiberFifoPut(chan, data);
}

/****************************************************************************/

static __inline__ void * nanoTaskLifoGet(NANO_LIFO * chan) {

	return NULL;
}

static __inline__ void * nanoTaskLifoGetW(NANO_LIFO * chan) {

	return NULL;
}

static __inline__ void nanoTaskLifoPut(NANO_LIFO * chan, void * data) {

}

/****************************************************************************/

static __inline__ void nanoTaskSemGive(NANO_SEM * chan) {
	mcos_signel(__viper->mInfo, chan->sem);
}

static __inline__ void nanoTaskSemTake(NANO_SEM * chan) {
	mcos_wait(__viper->mInfo, chan->sem);
}

static __inline__ void nanoTaskSemTakeW(NANO_SEM * chan) {
	mcos_try(__viper->mInfo, chan->sem);
}

/****************************************************************************/

static __inline__ int nanoTaskStackPop(NANO_STACK * chan, uint32_t * pData) {

	return nanoFiberLifoGet(chan, pData);
}

static __inline__ uint32_t nanoTaskStackPopW(NANO_STACK * chan, uint32_t * pData) {

	return nanoFiberLifoGetW(chan, pData);
}

static __inline__ void nanoTaskStackPush(NANO_STACK * chan, uint32_t data) {

	nanoFiberLifoPut(chan, &data);
}

/****************************************************************************/

static __inline__ void * nanoUserPtrGet(void) {

	return __viper->mInfo->curr->priv;
}

static __inline__ void nanoUserPtrSet(void * data) {

	__viper->mInfo->curr->priv = data;
}

/****************************************************************************/

#endif /* rte_viper_H_ */
