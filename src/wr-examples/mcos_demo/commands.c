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

/*
 * Copyright (c) 2009, Olivier MATZ <zer0@droids-corp.org>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
/* Created 2010 by Keith Wiles @ windriver.com */
#include <rte_config.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <string.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <termios.h>
#include <fcntl.h>

#include <rte_log.h>
#include <rte_tailq.h>
#include <rte_memory.h>
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

#include <cmdline_rdline.h>
#include <cmdline_parse.h>
#include <cmdline_socket.h>
#include <cmdline_parse_string.h>
#include <cmdline_parse_num.h>
#include <cmdline.h>

#include "rte_mcos.h"
#include "mcos_main.h"
#include "commands.h"

char *
cmd_port_display(char * buff, uint32_t len, uint32_t portlist) {
	
	char	  * p = buff;
	uint32_t	bit = 0, first, comma = 0;
	
	buff[0] = '\0';
	while( bit < (sizeof(portlist) << 3) ) {
		if ( (portlist & (1 << bit++)) == 0 )
			continue;
		first = (bit-1);
		while( (portlist & (1 << bit)) )
			bit++;
		if ( first == (bit-1) )
			snprintf(p, len - strlen(buff), "%s%d", (comma)?",":"", first);
		else
			snprintf(p, len - strlen(buff), "%s%d-%d", (comma)?",":"", first, (bit-1));

		p = buff + strlen(buff);
		if ( strlen(buff) > (len - 5) )
			break;
		comma = 1;
	}
	return buff;
}

void
cmdline_pause(struct cmdline *cl, const char * msg)
{
	char c;
	int	n;
	
	cmdline_printf(cl, "%s", msg);
	n = read(cl->s_in, &c, 1);
	if ( n < 0 )
		return;
	cmdline_printf(cl, "\r");
	scrn_eol();
}

/**********************************************************/
const char * help_info[] = {
		"", /* Leave blank not used */
		"      *** Help Information for MCOS " MCOS_DEMO_VERSION " ***\n",
		"master                             - Show Master information\n"
		"checkstack N                       - Show all fiber stack information for the given MCOS instance\n"
		"instance N                         - List all instances info or a given instance\n"
		"list N                             - List all of the fibers for lcore (N)\n"
		"info N                             - Show the lcore (N) information\n",
		"ready N                            - Show the lcore (N) ready queue\n",
		"sleep N                            - Show the lcore (N) sleep queue\n",
		"clear                              - Clear the statistics\n",
		"cls                                - Clear the screen\n",
		"reset                              - Reset the configuration to the default\n",
		"quit                               - Quit the mcos program\n",
	NULL
};

struct cmd_help_result {
	cmdline_fixed_string_t help;
};

static void cmd_help_parsed(__attribute__((unused)) void *parsed_result,
			    struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	int		i;
	
	scrn_pause();
	scrn_setw(1);
	scrn_cls();
	
	for(i=1; help_info[i] != NULL; i++ ) {
		cmdline_printf(cl, help_info[i]);
		if ( (i % (MAX_SCRN_ROWS-3)) == 0 )
			cmdline_pause(cl, "   <More Help: Press Return to Continue>");
	}

	cmdline_pause(cl, "   <Press Return to Continue>");
	scrn_setw(mcos.last_row+1);
	
	scrn_resume();
	
	mcos_redisplay(1);
}

cmdline_parse_token_string_t cmd_help_help =
	TOKEN_STRING_INITIALIZER(struct cmd_help_result, help, "help");

cmdline_parse_inst_t cmd_help = {
	.f = cmd_help_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "show help",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_help,
		NULL,
	},
};

/**********************************************************/
struct cmd_master_result {
	cmdline_fixed_string_t master;
};

static void cmd_master_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	mcos_list_master();
}

cmdline_parse_token_string_t cmd_help_master =
	TOKEN_STRING_INITIALIZER(struct cmd_master_result, master, "master");

cmdline_parse_inst_t cmd_master = {
	.f = cmd_master_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "List master instances",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_master,
		NULL,
	},
};

/**********************************************************/

struct cmd_clear_result {
	cmdline_fixed_string_t clear;
};

static void cmd_clear_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	mcos_clear_stats();
}

cmdline_parse_token_string_t cmd_help_clear =
	TOKEN_STRING_INITIALIZER(struct cmd_clear_result, clear, "clear");

cmdline_parse_inst_t cmd_clear = {
	.f = cmd_clear_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "Clear statistics",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_clear,
		NULL,
	},
};

/**********************************************************/

struct cmd_quit_result {
	cmdline_fixed_string_t quit;
};

static void cmd_quit_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	mcos_quit();
}

cmdline_parse_token_string_t cmd_help_quit =
	TOKEN_STRING_INITIALIZER(struct cmd_quit_result, quit, "quit");

cmdline_parse_inst_t cmd_quit = {
	.f = cmd_quit_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "Quit the MCOS program",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_quit,
		NULL,
	},
};

/**********************************************************/

struct cmd_info_result {
	cmdline_fixed_string_t info;
	uint16_t	lcore;
};

static void cmd_info_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_info_result *res = parsed_result;

	mcos_show_info(res->lcore);
}

cmdline_parse_token_string_t cmd_help_info =
	TOKEN_STRING_INITIALIZER(struct cmd_info_result, info, "info");
cmdline_parse_token_num_t cmd_info_lcore =
	TOKEN_NUM_INITIALIZER(struct cmd_info_result, lcore, UINT16);

cmdline_parse_inst_t cmd_info = {
	.f = cmd_info_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "info N - Print Information about internal structures.",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_info,
		(void *)&cmd_info_lcore,
		NULL,
	},
};

/**********************************************************/

struct cmd_stack_check_result {
	cmdline_fixed_string_t checkstack;
	uint16_t	idx;
};

static void cmd_stack_check_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_stack_check_result *res = parsed_result;

	mcos_show_stackcheck(res->idx);
}

cmdline_parse_token_string_t cmd_help_stack_check =
	TOKEN_STRING_INITIALIZER(struct cmd_stack_check_result, checkstack, "checkstack");
cmdline_parse_token_num_t cmd_stack_check_idx =
	TOKEN_NUM_INITIALIZER(struct cmd_stack_check_result, idx, UINT16);

cmdline_parse_inst_t cmd_stack_check = {
	.f = cmd_stack_check_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "checkstack N - Print Information about all fibers",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_stack_check,
		(void *)&cmd_stack_check_idx,
		NULL,
	},
};

/**********************************************************/

struct cmd_ready_result {
	cmdline_fixed_string_t ready;
	uint16_t	lcore;
};

static void cmd_ready_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_ready_result *res = parsed_result;

	mcos_show_ready(res->lcore);
}

cmdline_parse_token_string_t cmd_help_ready =
	TOKEN_STRING_INITIALIZER(struct cmd_ready_result, ready, "ready");
cmdline_parse_token_num_t cmd_ready_lcore =
	TOKEN_NUM_INITIALIZER(struct cmd_ready_result, lcore, UINT16);

cmdline_parse_inst_t cmd_ready = {
	.f = cmd_ready_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "ready N - Print Information about ready list.",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_ready,
		(void *)&cmd_ready_lcore,
		NULL,
	},
};

/**********************************************************/

struct cmd_sleep_result {
	cmdline_fixed_string_t sleep;
	uint16_t	lcore;
};

static void cmd_sleep_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_sleep_result *res = parsed_result;

	mcos_show_sleep(res->lcore);
}

cmdline_parse_token_string_t cmd_help_sleep =
	TOKEN_STRING_INITIALIZER(struct cmd_sleep_result, sleep, "sleep");
cmdline_parse_token_num_t cmd_sleep_lcore =
	TOKEN_NUM_INITIALIZER(struct cmd_sleep_result, lcore, UINT16);

cmdline_parse_inst_t cmd_sleep = {
	.f = cmd_sleep_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "sleep N - Print Information about sleep list.",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_sleep,
		(void *)&cmd_sleep_lcore,
		NULL,
	},
};

/**********************************************************/

struct cmd_list_result {
	cmdline_fixed_string_t list;
	uint16_t	lcore;
};

static void cmd_list_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_list_result *res = parsed_result;

	mcos_show_fibers(res->lcore);
}

cmdline_parse_token_string_t cmd_help_list =
	TOKEN_STRING_INITIALIZER(struct cmd_list_result, list, "list");
cmdline_parse_token_num_t cmd_list_lcore =
	TOKEN_NUM_INITIALIZER(struct cmd_list_result, lcore, UINT16);

cmdline_parse_inst_t cmd_list = {
	.f = cmd_list_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "list N - Print Information about fiber list.",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_list,
		(void *)&cmd_list_lcore,
		NULL,
	},
};

/**********************************************************/

struct cmd_instance_result {
	cmdline_fixed_string_t instance;
	uint16_t	idx;
};

static void cmd_instance_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	struct cmd_instance_result *res = parsed_result;

	mcos_show_instances(res->idx);
}

cmdline_parse_token_string_t cmd_help_instance =
	TOKEN_STRING_INITIALIZER(struct cmd_instance_result, instance, "instance");
cmdline_parse_token_num_t cmd_instance_idx =
	TOKEN_NUM_INITIALIZER(struct cmd_instance_result, idx, UINT16);

cmdline_parse_inst_t cmd_instance = {
	.f = cmd_instance_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "instance N - Print Information about an instance.",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_instance,
		(void *)&cmd_instance_idx,
		NULL,
	},
};

/**********************************************************/

struct cmd_cls_result {
	cmdline_fixed_string_t cls;
};

static void cmd_cls_parsed(__attribute__((unused)) void *parsed_result,
				__attribute__((unused)) struct cmdline *cl,
			    __attribute__((unused)) void *data)
{
	mcos_redisplay(1);
}

cmdline_parse_token_string_t cmd_help_cls =
	TOKEN_STRING_INITIALIZER(struct cmd_cls_result, cls, "cls");

cmdline_parse_inst_t cmd_cls = {
	.f = cmd_cls_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = "Clear screen",
	.tokens = {        /* token list, NULL terminated */
		(void *)&cmd_help_cls,
		NULL,
	},
};

/**********************************************************/
/**********************************************************/
/****** CONTEXT (list of instruction) */

cmdline_parse_ctx_t main_ctx[] = {
		(cmdline_parse_inst_t *)&cmd_master,
		(cmdline_parse_inst_t *)&cmd_stack_check,
		(cmdline_parse_inst_t *)&cmd_instance,
		(cmdline_parse_inst_t *)&cmd_info,
		(cmdline_parse_inst_t *)&cmd_ready,
		(cmdline_parse_inst_t *)&cmd_sleep,
		(cmdline_parse_inst_t *)&cmd_list,
		(cmdline_parse_inst_t *)&cmd_cls,
		(cmdline_parse_inst_t *)&cmd_quit,
		(cmdline_parse_inst_t *)&cmd_help,
		NULL,
};
