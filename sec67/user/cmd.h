/*
 * Copyright: (c) 2006-2007, 2008 Triductor Technology, Inc.
 * All Rights Reserved.
 *
 * File:		cmd.h
 * Purpose: 	Command function prototypes.
 * History: 	4/30/2007, created by jetmotor.
 */

#ifndef _CMD_H
#define _CMD_H

#include "vsh.h"
extern void cmd_init(view_t sview[], u32 sview_nm, command_t scmd[], u32 scmd_nm);

extern void docmd_exit(command_t *cmd, xsh_t *xsh);

extern void (*esprintf)(const char *fmt, ...);

#if 0														// demo
#include "cmd.h"
param_t cmd_demo_param_tab[] = {
	{ PARAM_ARG | PARAM_TOGGLE, "", "ptr"
	},
	PARAM_EMPTY
};

cmd_desc_t cmd_demo_desc = CMD_DESC_ENTRY(
						  "", cmd_demo_param_tab
);

void docmd_demo(command_t *cmd, xsh_t *xsh)
{
	if (strcmp(xsh->argv[1], "ptr") == 0)
	{
		esprintf("test success\n");
	}

	return;
}

view_t g_demoview[] =
{
	VIEWTAB_ENTRY(demo,   "[demo /] "),
};

command_t g_democmd[] =
{
	CMDTAB_ENTRY(demo,		 root,  demo,    docmd_exit,      	  NULL),

	CMDTAB_ENTRY(exit,		 demo, root,     docmd_exit,      	  NULL),
	CMDTAB_ENTRY(..,		 demo, root,     docmd_exit,      	  NULL),
	CMDTAB_ENTRY(read,		 demo, demo,    docmd_demo,   	  &cmd_demo_desc),
};

void demo_init(void)
{
	u32 v_num, c_num;

	v_num = sizeof(g_demoview) / sizeof(g_demoview[0]);
	c_num = sizeof(g_democmd) / sizeof(g_democmd[0]);

	cmd_init(g_demoview, v_num, g_democmd, c_num);
}

#endif

void sec_cmd_init(void);

#endif	/* end of _CMD_H */

