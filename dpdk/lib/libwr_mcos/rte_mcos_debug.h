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

#ifndef __MCOS_DEBUG_H
#define __MCOS_DEBUG_H

#define MAX_DBG_MESSAGE		256
#define DBG_BUF_SIZE		256

#if (INCLUDE_DEBUG_SUPPORT == 1)

#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 7)
#define dbgDeclare( ... )       __VA_ARGS__;		/**< Declare a debug variable */
#else
#define dbgDeclare( param )     param;				/**< Declare a debug variable */
#endif /* !(__GNUC__ == 2 && __GNUC_MINOR__ == 7) */

/*lint -emacro( {717}, dbgLine ) */
#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 7)
#define dbgLine( ... )     /**< Include a debug line of code */         \
        do {                                                            \
            __VA_ARGS__;  	                                            \
        } while( (0) )
#else
#define dbgLine( params... )    /**< Include a debug line of code */    \
        do {                                                            \
            ##params;                                                   \
        } while( (0) )
#endif /* GNU_COMPILER_272 */
        
/*lint -emacro( {717}, dbgPrintf ) */
#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 7)
#define dbgPrintf( ... )         /**< Print a message using printf */             			\
            dbgPrintInfo(mInfo, __PRETTY_FUNCTION__, __FILE__, __LINE__,     				\
                    (snprintf((char *)dbgBuf, DBG_BUF_SIZE, __VA_ARGS__), dbgBuf))
#else
#define dbgPrintf( params... )   /**< Print a message using printf */      		\
            dbgPrintInfo(__FUNCTION__, __FILE__, __LINE__,     					\
                    (snprintf((char *)dbgBuf, DBG_BUF_SIZE, ##params), dbgBuf))
#endif /* GNU_COMPILER_272 */

/*lint -emacro( {717}, dbgInfo ) */
#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 7)
#define dbgInfo( ... )           /**< Print a information message */       		\
            dbgPrintInfo(__PRETTY_FUNCTION__, __FILE__, __LINE__,    			\
                (snprintf((char *)dbgBuf, DBG_BUF_SIZE, __VA_ARGS__), dbgBuf))
#else
#define dbgInfo( params... )     /**< Print a information message */       	\
            dbgPrintInfo(__FUNCTION__, __FILE__, __LINE__,         			\
                (snprintf((char *)dbgBuf, DBG_BUF_SIZE, ##params), dbgBuf))
#endif /* !(__GNUC__ == 2 && __GNUC_MINOR__ == 7) */


#else

#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 7)
#define dbgDeclare( ... )       /* */			/**< Declare a debug variable */
#define dbgLine( ... )          E_NOOP			/**< Include a debug line of code */
#define dbgInfo( ... )          E_NOOP			/**< Print a information message */
#define dbgPrintf( ... )        E_NOOP			/**< Print a message using printf */
#else
#define dbgDeclare( params... )     /* */		/**< Declare a debug variable */
#define dbgLine( params... )        E_NOOP		/**< Include a debug line of code */
#define dbgInfo( params... )        E_NOOP		/**< Print a information message */
#define dbgPrintf( params... )      E_NOOP		/**< Print a message using printf */
#endif /* GNU_COMPILER_272 */

#endif      /* INCLUDE_DEBUG_SUPPORT */

#if (INCLUDE_ASSERT_SUPPORT == 1)
#define dbgAssert(expr)					/**< Debug assert macro */				\
			if ( !(expr) )														\
				dbgAssertInfo(mk_string(expr), __PRETTY_FUNCTION__, __FILE__, __LINE__)

#else
#define dbgAssert(expr)			E_NOOP	/**< Debug assert macro */
#endif

/**************************************************************************//**
* Setup the debug system and set memory to the correct initial value.
*
* \param log The flag is to enable logging.
*
* \returns E_OK
*
* ERRNO: N/A
*/
extern status_t mcos_dbgInit( int32_t log );

/**************************************************************************//**
*
* Dump out a list or queue of information.
*
* \param mInfo The MCOS instance structure.
* \param msg The message the to be displayed.
* \param head The head of the queue to dump.
*
* \returns N/A
*
* SEE ALSO:
*/
extern void		mcos_dbgListq(mInfo_t * mInfo, c8_t * msg, int16_t head);

/**************************************************************************//**
*
* Dump out a list or queue of information.
*
* \param mInfo The MCOS instance structure.
* \param msg The message the to be displayed.
* \param head The head of the queue to dump.
*
* \returns N/A
*
* SEE ALSO:
*/
extern void		mcos_dbgLists(mInfo_t * mInfo, c8_t * msg, int16_t head);

/**************************************************************************//**
* A routine called from dbgPrintf() macro to print a debug message on the
* console port. The output displays the message plus pre-pending task name,
* file name, function name and line number to the message.
*
* The routine is called with a set of values and a message string.
*
* \param mInfo	The MCOS instance structure.
* \param func	String pointer of the function from __FUNCTION__ macro.
* \param file	String pointer of the filename __FILE__ macro.
* \param line	A 32 bit integer of the line number __LINE__ the message statement appeared.
* \param pBuf   The buffer to be displayed.
*
* \returns N/A.
*
* ERRNO: N/A
*/
extern void     dbgPrintInfo(mInfo_t * mInfo, c8_t * file, c8_t * func, const int32_t line, int8_t * pBuf );

/**************************************************************************//**
*
* Handle the dbgAssert and print out a message.
*
* \param expr   The expression string to display.
* \param func	String pointer of the function from __FUNCTION__ macro.
* \param file	String pointer of the filename __FILE__ macro.
* \param line	A 32 bit integer of the line number __LINE__ the message statement appeared.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void		dbgAssertInfo(c8_t * expr, c8_t * file, c8_t * func, const int32_t line);

// External routines
/**************************************************************************//**
*
* List the Master information instance data.
*
* \param master The master MCOS structure pointer.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void		mcos_master_list( mcos_master_t * master );

/**************************************************************************//**
*
* Find a instance id matching the given name.

* \param master The master MCOS structure pointer.
* \param idx The instance index value to list.
*
* \returns E_ERROR or instance_id
*
* SEE ALSO:
*/

extern uint32_t mcos_list_instance_id(mcos_master_t * master, uint8_t idx);

/**************************************************************************//**
*
* Dump out a list or queue of information.
*
* \param mInfo	The MCOS instance structure.
* \param msg	The message to display.
* \param head	The head of the list to display.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void		mcos_listq(mInfo_t * mInfo, c8_t * msg, int16_t head);

/**************************************************************************//**
*
* Dump out some information about mcos to the console.
*
* \param mInfo	The MCOS instance structure.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void		mcos_info(mInfo_t * mInfo);

/**************************************************************************//**
*
* List all fibers of a MCOS instance
*
* \param mInfo	The MCOS instance structure.
*
* List all fibers of an instance.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void		mcos_list_fibers(mInfo_t * mInfo);

/**************************************************************************//**
*
* List all fibers of a MCOS instance with its stack usage
*
* \param mInfo	The MCOS instance structure.
*
* List all fibers of an instance with its stack usage information.
*
* \returns N/A
*
* SEE ALSO:
*/

extern void mcos_stack_check(mInfo_t * mInfo);

extern int8_t * dbgBuf;				//!< global debug printf buffer

#endif
