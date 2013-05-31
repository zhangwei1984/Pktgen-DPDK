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

#ifndef mcos_FIFO_H_
#define mcos_FIFO_H_

typedef struct fifo_s {
	int16_t		size;				/**< Size of fifo in entries */
	int16_t		count;				/**< current number of entries */
	int16_t		head;				/**< Head of FIFO */
	int16_t		tail;				/**< Tail of FIFO */
	int16_t		rsem;				/**< read semaphore ID */
	int16_t		wsem;				/**< write semaphore ID */
	uint32_t	flags;				/**< Flags for FiFo */
	data_u    * data;				/**< FIFO entries */
} fifo_t __attribute__((__aligned__(CACHE_LINE_SIZE)));		/**< FIFO information structure */

/**************************************************************************//**
*
* Create a FIFO of the given length.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] addr Pointer to memory to use, if NULL allocate it.
* \param [in] size Size of the fifo.
*
* \return NULL or the pointer to the the fifo structure.
*
* SEE ALSO:
*/

static __inline__ fifo_t * mcos_fcreate(mInfo_t * mInfo, void * addr, int16_t size) {

	fifo_t	  * fifo = NULL;

	fifo = (fifo_t *)rte_calloc("fifo", 1, sizeof(fifo_t), 0);
	if ( fifo != NULL ) {
		if ( addr == NULL ) {
			fifo->data = (data_u *)rte_calloc("fdata", size, sizeof(data_u), 0);
			if ( fifo->data == NULL ) {
				rte_free((void *)fifo);
				return NULL;
			}
			fifo->flags = ALLOCATED_MEMORY_FLAG;
		} else
			fifo->data = addr;

		fifo->size		= size;
		fifo->rsem		= mcos_screate(mInfo, 0);
		fifo->wsem		= mcos_screate(mInfo, 0);
		fifo->tail		= 0;
		fifo->head		= size;
		if ( (fifo->rsem == E_ERROR) || (fifo->wsem == E_ERROR) ) {
			mcos_sdelete(mInfo, fifo->rsem);
			mcos_sdelete(mInfo, fifo->wsem);
			if ( fifo->flags & ALLOCATED_MEMORY_FLAG )
				rte_free((void *)fifo->data);
			rte_free((void *)fifo);
			return NULL;
		}
	}

	return fifo;
}

/**************************************************************************//**
*
* Destroy the fifo and return the memory.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] fifo Pointer to the allocated fifo structure.
*
* \return N/A
*
* SEE ALSO:
*/

static __inline__ void mcos_fdestory(mInfo_t * mInfo, fifo_t * fifo) {

	if ( fifo ) {
		mcos_sdelete(mInfo, fifo->rsem);
		mcos_sdelete(mInfo, fifo->wsem);
		if ( fifo->flags & ALLOCATED_MEMORY_FLAG )
			rte_free((void *)fifo->data);
		rte_free((void *)fifo);
	}
}

/**************************************************************************//**
*
* Internal function to allocate a structure to push onto a fifo.
*
* \param [in] fifo Pointer to the allocated fifo structure.
*
* \return NULL on ERROR or fentry_u pointer to put data
*
* SEE ALSO:
*/

static __inline__ data_u * __falloc(fifo_t * fifo) {
	void	  * data = NULL;

	if ( fifo && (fifo->count < fifo->size) ) {
		data = &fifo->data[fifo->tail++];
		fifo->count++;
		if ( fifo->tail >= fifo->size )
			fifo->tail = 0;
	}

	return data;
}

/**************************************************************************//**
*
* Internal function to pop data off a fifo
*
* \param [in] fifo Pointer to the allocated fifo structure.
*
* \return NULL on ERROR or data_u pointer
*
* SEE ALSO:
*/

static __inline__ void * __fpop(fifo_t * fifo ) {
	void	  * data = NULL;

	if ( fifo && (fifo->count > 0) ) {
		if ( fifo->head >= fifo->size )
			fifo->head = 0;
		data = fifo->data[fifo->head++].pdata;
		fifo->count--;
	}
	return data;
}

/**************************************************************************//**
*
* Push the data onto a fifo.
*
* \param [in] fifo Pointer to the allocated fifo structure.
* \param [in] p Pointer to the data_u union for data.
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_fpush(fifo_t * fifo, void * p) {

	data_u	  * data = __falloc(fifo);

	if ( data == NULL )
		return E_ERROR;
	data->pdata = p;

	return E_OK;
}

/**************************************************************************//**
*
* Pop data from a fifo but do not wait if empty
*
* \param [in] fifo Pointer to the allocated fifo structure.
* \param [out] p Pointer to the data_u union for data.
*
* \return E_ERROR or E_OK with data placed in <p>
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_fpop(fifo_t * fifo, void ** p) {
	void	  * data = __fpop(fifo);

	if ( data == NULL )
		return E_ERROR;
	*p = data;

	return E_OK;
}

/**************************************************************************//**
*
* Push data to a fifo and wait if full.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] fifo Pointer to the allocated fifo structure.
* \param [in] p Pointer to the data.
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_fpush_wait(mInfo_t * mInfo, fifo_t * fifo, void * pData) {

	do {
		if ( mcos_fpush(fifo, &pData) == E_OK )
			break;
		mcos_wait(mInfo, fifo->wsem);
	} while(1);

	mcos_signal(mInfo, fifo->rsem);

	return E_OK;
}

/**************************************************************************//**
*
* Pop data from a fifo and wait if empty.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] fifo Pointer to the allocated fifo structure.
* \param [out] p Pointer to the data.
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_fpop_wait(mInfo_t * mInfo, fifo_t * fifo, void * p) {

	do {
		if ( mcos_fpop(fifo, &p) == E_OK )
			break;
		mcos_wait(mInfo, fifo->rsem);
	} while(1);

	mcos_signal(mInfo, fifo->wsem);

	return E_OK;
}

#endif /* mcos_FIFO_H_ */
