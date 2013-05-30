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
 * 4) The screens displayed by the application must contain the copyright notice as defined
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

#include <rte_scrn.h>

#ifndef _MCOS_IPC_H_
#define _MCOS_IPC_H_

extern int main(int argc, char **argv);

#define MCOS_IPC_VERSION		"1.0.0"
#define COPYRIGHT_MSG			"Copyright (c) <2010-2012>, Wind River Systems, Inc."
#define POWERED_BY_DPDK			"Powered by Intel® DPDK"

// A couple of options to enable if needed

#define MAX_MATRIX_ENTRIES      128
#define MAX_STRING              256
#define iBitsTotal(_x) \
    (((_x.ipackets * (INTER_FRAME_GAP + PKT_PREAMBLE_SIZE)) + _x.ibytes) << 3)
#define oBitsTotal(_x) \
    (((_x.opackets * (INTER_FRAME_GAP + PKT_PREAMBLE_SIZE)) + _x.obytes) << 3)

enum {
	MAX_SCRN_ROWS			= 43,
	MAX_SCRN_COLS			= 132,

	COLUMN_WIDTH_0			= 18,
	COLUMN_WIDTH_1			= 22,
	MAX_DEMO_THREADS		= 16
};

#define Million			(uint64_t)(1000ULL * 1000ULL)
#define Mega			(uint64_t)(1024ULL * 1024ULL)

typedef struct mcos_s {
	struct cmdline		  * cl;					/**< Command Line information pointer */
	rte_scrn_t			  * scrn;				/**< Pointer to scrn information */
	uint32_t				flags;				/**< Flag values */
	uint16_t				last_row;			/**< last static row of the screen */
	uint8_t					map_idx2lcores[RTE_MAX_LCORE];
	uint8_t					nb_lcores;
	uint8_t					master_lcore;
	uint64_t				hz;					/**< Number of events per seconds */
	uint64_t				clk[3];
} mcos_t;

enum {		// mcos flags bits
	PRINT_LABELS_FLAG		= 0x00000001,		/**< Print constant labels on stats display */
};

/**
 * Function returning string of version number: "- Version:x.y.x (DPDK-x.y.z)"
 * @return
 *     string
 */
static inline const char *
mcos_ipc_version(void) {
	return "Ver:"MCOS_IPC_VERSION"(DPDK-"
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
mcos_ipc_copyright(void) {
	return COPYRIGHT_MSG;
}

#define printf_info(...)	scrn_fprintf(0,0,stdout, __VA_ARGS__)

extern int	mcos_load_cmds( char * filename );
extern void mcos_redisplay( int cls_flag );
extern void mcos_clear_stats(void);
extern void mcos_quit(void);
extern void	mcos_show_info(uint16_t lcore);
extern void mcos_show_ready(uint16_t lcore);
extern void	mcos_show_sleep(uint16_t lcore);
extern void	mcos_show_fibers(uint16_t lcore);
extern void mcos_list_master(void);
extern void mcos_show_instances(uint16_t idx);
extern void mcos_show_ring(uint16_t lcore, uint16_t pid);
extern void mcos_set_fiber_count( uint16_t value );
extern void mcos_start_sending(uint32_t bitmap);
extern void mcos_stop_sending(uint32_t bitmap);
extern void mcos_show_stackcheck(uint16_t lcore);
extern void mcos_screen(const char * onOff);

extern mcos_t		mcos;

#endif /* _MCOS_IPC_H_ */
