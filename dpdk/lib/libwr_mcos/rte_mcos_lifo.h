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

#ifndef mcos_LIFO_H_
#define mcos_LIFO_H_

typedef struct lifo_s {
	int16_t		size;				/**< Size of lifo in entries */
	int16_t		count;				/**< current number of entries */
	int16_t		rsem;				/**< read semaphore ID */
	int16_t		wsem;				/**< write semaphore ID */
	uint32_t	flags;				/**< LiFo flags */
	uint32_t	pad;
	data_u	  * data;				/**< LIFO entries */
} lifo_t __attribute__((__aligned__(CACHE_LINE_SIZE)));		/**< LIFO information structure */

/**************************************************************************//**
*
* Create a LIFO of the given length.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] addr Pointer to memory to use, if NULL allocate it.
* \param [in] size Size of the lifo.
*
* \return NULL or the pointer to the the lifo structure.
*
* SEE ALSO:
*/

static __inline__ lifo_t * mcos_lcreate(mInfo_t * mInfo, void * addr, int16_t size) {

	lifo_t	  * lifo = NULL;

	lifo = (lifo_t *)rte_calloc("lifo", 1, sizeof(lifo_t), 0);
	if ( lifo != NULL ) {
		if ( addr == NULL ) {
			lifo->data = (data_u *)rte_calloc("ldata", size, sizeof(data_u), 0);
			if ( lifo->data == NULL ) {
				rte_free((void *)lifo);
				return NULL;
			}
			lifo->flags = ALLOCATED_MEMORY_FLAG;
		} else
			lifo->data = addr;

		lifo->size		= size;
		lifo->rsem		= mcos_screate(mInfo, 0);
		lifo->wsem		= mcos_screate(mInfo, 0);
		if ( (lifo->rsem == E_ERROR) || (lifo->wsem == E_ERROR) ) {
			mcos_sdelete(mInfo, lifo->rsem);
			mcos_sdelete(mInfo, lifo->wsem);
			if ( lifo->flags & ALLOCATED_MEMORY_FLAG )
				rte_free((void *)lifo->data);
			rte_free((void *)lifo);
			return NULL;
		}
	}

	return lifo;
}

/**************************************************************************//**
*
* Destroy the lifo and return the memory.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] lifo Pointer to the allocated lifo structure.
*
* \return N/A
*
* SEE ALSO:
*/

static __inline__ void mcos_ldestory(mInfo_t * mInfo, lifo_t * lifo) {

	if ( lifo ) {
		mcos_sdelete(mInfo, lifo->rsem);
		mcos_sdelete(mInfo, lifo->wsem);
		if ( lifo->flags & ALLOCATED_MEMORY_FLAG )
			rte_free((void *)lifo->data);
		rte_free((void *)lifo);
	}
}

/**************************************************************************//**
*
* Internal function to allocate a structure to push onto a lifo.
*
* \param [in] lifo Pointer to the allocated lifo structure.
*
* \return NULL on ERROR or lentry_u pointer to put data
*
* SEE ALSO:
*/

static __inline__ data_u * __lalloc(lifo_t * lifo) {

	if ( lifo && (lifo->count < lifo->size) )
		return &lifo->data[lifo->count++];
	else
		return NULL;
}

/**************************************************************************//**
*
* Internal function to pop data off a lifo
*
* \param [in] lifo Pointer to the allocated lifo structure.
*
* \return NULL on ERROR or lentry_u pointer
*
* SEE ALSO:
*/

static __inline__ void * __lpop(lifo_t * lifo ) {

	if ( lifo && (lifo->count > 0) )
		return lifo->data[--lifo->count].pdata;
	else
		return NULL;
}

/**************************************************************************//**
*
* Push the data structure lentry_u onto a lifo.
*
* \param [in] lifo Pointer to the allocated lifo structure.
* \param [in] p Pointer to the void pointer
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_lpush(lifo_t * lifo, void * pData) {

	data_u	  * data = __lalloc(lifo);

	if ( data == NULL )
		return E_ERROR;
	data->pdata = pData;

	return E_OK;
}

/**************************************************************************//**
*
* Pop data from a lifo but do not wait if empty
*
* \param [in] lifo Pointer to the allocated lifo structure.
* \param [out] p Pointer to the lentry_u union for data.
*
* \return E_ERROR or E_OK with data placed in <p>
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_lpop(lifo_t * lifo, void ** pData) {
	void	  * data = __lpop(lifo);

	if ( data == NULL )
		return E_ERROR;
	*pData = data;

	return E_OK;
}

/**************************************************************************//**
*
* Push data to a lifo and wait if full.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] lifo Pointer to the allocated lifo structure.
* \param [in] p Pointer to the lentry_u union for data.
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_lpush_wait(mInfo_t * mInfo, lifo_t * lifo, void * pData) {

	do {
		if ( mcos_lpush(lifo, &pData) == E_OK )
			break;
		mcos_wait(mInfo, lifo->wsem);
	} while(1);

	mcos_signal(mInfo, lifo->rsem);

	return E_OK;
}

/**************************************************************************//**
*
* Pop data from a lifo and wait if empty.
*
* \param [in] mInfo Pointer to the MCOS structure.
* \param [in] lifo Pointer to the allocated lifo structure.
* \param [out] p Pointer to the lentry_u union for data.
*
* \return E_ERROR or E_OK
*
* SEE ALSO:
*/

static __inline__ int32_t mcos_lpop_wait(mInfo_t * mInfo, lifo_t * lifo, void * p) {

	do {
		if ( mcos_lpop(lifo, &p) == E_OK )
			break;
		mcos_wait(mInfo, lifo->rsem);
	} while(1);

	mcos_signal(mInfo, lifo->wsem);

	return E_OK;
}

#endif /* mcos_LIFO_H_ */
