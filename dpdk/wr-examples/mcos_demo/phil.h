/*-
 * Copyright (c) <2010>, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of Intel Corporation nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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

#ifndef __INC_PHIL_H
#define __INC_PHIL_H

/* Defines */
#define TRUE			  1
#define FALSE			  0

#define FORK_AVAILABLE    0
#define FORK_IN_USE       1

#define NOBODY           -1

#define CLAIM_BASED       1
#define TICKET_BASED      2

#define PRINT_LOCK_ACQUIRE   do { scrn_save(); } while((0))
#define PRINT_LOCK_RELEASE   do { fflush(stdout); scrn_restore(); } while((0))

typedef int32_t		BOOL;
/* externs */

/* Typedefs */

typedef struct philosophers_attributes
    {
    int numPhil;	/* number of philosophers */
    int16_t syncSemId;	/* Synchronization semaphore */
    int16_t pad0;
    int16_t	* pPhil;	/* Points to an array of philosopher task IDs */
    BOOL * iAmReady;	/* Array collecting philosopher's ready status */
    } PHILOSOPHERS_ATTR;

typedef struct fork
    {
    int status;         /* FORK_AVAILABLE or FORK_IN_USE */
    int philosopher;    /* Philosopher requesting the fork */
    } A_FORK;

typedef struct forks_attributes
    {
    int numForks;       /* number of forks */
    uint16_t forkMutex;   /* Protection mutex for the fork resource */
    int16_t	pad0;
    A_FORK * forks;     /* The set of forks on the table */
    } FORKS_ATTR;

typedef struct statictics
    {
    unsigned int  * goHungry;    /* The number of times a philosopher waits */
    unsigned int  * foodIntake;  /* the number of times a philosopher eats */
    } STATS;

typedef struct info_s {
	unsigned int		duration;
	unsigned int        solution;       /* Either one of the impl. solutions */
	PHILOSOPHERS_ATTR * philosAttr;     /* Philosopher attributes */
	FORKS_ATTR        * forksAttr;      /* Fork attributes */
	STATS             * stats;          /* Statistic data */
	int                 idx;            /* Loop counter */
	int					doing;
} info_t;

typedef struct saved_s {
	uint16_t	doing;
	uint16_t	count;
} saved_t;

/* locals */

/* Forward declarations */

void phil_demo_start(mInfo_t * mInfo, void * arg);
void philDemo(mInfo_t * mInfo, void * arg);
void philosopherRun (mInfo_t * mInfo, info_t * info);
BOOL forksGet(mInfo_t * mInfo, int myNum, int solution, FORKS_ATTR * pForksAttr, info_t * info);
void forksPut(mInfo_t * mInfo, int myNum, int solution, FORKS_ATTR * pForksAttr, info_t * info);
void phil_quit(void);

extern const char * doing_str[];

#define MAX_PHILS		15
extern info_t		  * infos[RTE_MAX_LCORE][MAX_PHILS];
extern saved_t			saved[RTE_MAX_LCORE][MAX_PHILS];

#endif /* __INC_PHIL_H */
