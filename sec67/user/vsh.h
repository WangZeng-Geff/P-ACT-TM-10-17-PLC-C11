/*
 * Copyright: (c) 2006-2007, 2008 Triductor Technology, Inc.
 * All Rights Reserved.
 *
 * File:	vsh.h
 * Purpose:	A very tiny shell skeleton.
 * History:
 *	Apr 30, 2007	jetmotor	Create
 */
 
#ifndef __VSH_H__
#define __VSH_H__


/********************************************************
 *		Notice: debug xsh or not?		*
 ********************************************************/
#define XSH_DEBUG


#include "rbtree.h"
#include "term.h"

#include <stdlib.h>


/**
 * view_t - Shell commands will be grouped by views. That is to say, the shell may have
 *	several views, each of which have several commands.
 *
 * @rb:		used as rb_tree link because views are organized as a rb_tree
 * @name:	view's name, used as key when search views in shell's view tree
 * @prompt:	view's prompt
 * @cmd_tr:	view's commands rb_tree
 */
typedef struct view_s{
	struct rb_node rb;

	char		*name;
	char		*prompt;
	rb_tree_t	cmd_tr;
}view_t;


#define VIEWTAB_ENTRY(name, prompt) \
	{RB_NODE_INIT(), #name, prompt, RB_TREE_INIT()}

extern view_t g_viewtab[];
extern rb_tree_t g_view_tr;


#define PARAM_NONE	0x00
/* properties for values of parameters
 */
#define PARAM_INTEGER	0x01
#define PARAM_NUMBER	0x02
#define PARAM_STRING	0x04
#define PARAM_MAC	0x08
#define PARAM_IP	0x10
#define PARAM_LIST	0x20
#define PARAM_OPTIONAL	0x40
#define PARAM_TOGGLE	0x80
#define PARAM_RANGE	0x100
#define PARAM_SUB	0x200
/* types for parameters
 */
#define PARAM_OPT	0x10000
#define PARAM_FLG	0x20000
#define PARAM_ARG	0x40000

#define PARAM_DONE	0x08000000

typedef struct param_s{
	uint32_t	type;
	char		*name;
	char		*help;
	char		*ref;
	struct list_head next;
	struct param_s	*list;
	struct param_s  *n;
}param_t;

#define param_info(param, info) do { \
        if ((param)->type & PARAM_TOGGLE) \
                (info) = (param)->help; \
        else if ((param)->type & PARAM_INTEGER) \
                (info) = "<int>"; \
        else if ((param)->type & PARAM_NUMBER) \
                (info) = "<num>"; \
        else if ((param)->type & PARAM_MAC) \
                (info) = "<mac>"; \
        else if ((param)->type & PARAM_IP) \
                (info) = "<ip>"; \
        else if ((param)->type & PARAM_STRING) \
                (info) = "<str>"; \
        else if ((param)->type & PARAM_RANGE) \
                (info) = "<range>"; \
} while (0)

#define PARAM_EMPTY		{PARAM_NONE, (char *)0, (char *)0, (char *)0, {0,0}, (param_t *)0, (param_t *)0}
extern param_t cmd_none_param_tab[];


typedef struct cmd_desc_s{
	char		*help;
	param_t		*param_tab;
	param_t		*list;
	param_t		*n;
}cmd_desc_t;

#define CMD_DESC_ENTRY(h, t)	{h, t, 0, 0}


struct command_s;
struct xsh_s;
/**
 * action_fp - Function pointer of command action.
 */
typedef void (*action_fp)(struct command_s *cmd, struct xsh_s *xsh);

/**
 * command_t - Each command corresponds to an action that an operator could type on a CLI
 *	interface to accomplish some operation. Each also has a current view and a next
 *	view. After invoking some command's action, shell's view may change from current
 *	view into the next view, such as `exit' etc..
 *
 * @rb:		used as rb_tree link because commands are organized as a rb_tree
 * @list:	matched commands list when shell perform tab expansion
 * @name:	command's name, used as key when search commands in view's command tree
 * @len:	length of name
 * @cview_nm:	current view name
 * @cview:	current view
 * @nview_nm:	next view name
 * @nview:	next view
 * @action:	command action function, invoked when shell matches this command
 */
typedef struct command_s{
	struct rb_node rb;
	struct list_head list;
	
	char		*name;
	char		*suffix;
	uint32_t	len;

	char		*cview_nm;
	view_t		*cview;
	char		*nview_nm;
	view_t		*nview;
	
	action_fp	action;
	cmd_desc_t	*desc;
}command_t;


#define CMDTAB_ENTRY(nm, cvnm, nvnm, action, desc) \
	{RB_NODE_INIT(), {0,0}, #nm, 0, 0, #cvnm, 0, #nvnm, 0, action, desc}


/**
 * histent_t - A place to record characters that term receives after each carriage return stroke 
 *	occurs. Notice that the buffer in hisent_t is of the same length with buffer in term_t.
 *
 * @link:	list link because all the histent_t are linked as a list
 * @line:	the buffer
 * @line_size:	actual data length in the buffer
 */
typedef struct histent_s{
	struct list_head link;
	
	char line[TTY_BUF_SIZE_MAX];
	int32_t line_size;
}histent_t;


#define NR_HISTENT_MAX		8
#define NR_ARG_MAX		32
#define TASK_STK_SIZE_XSH	1024*2


typedef struct exec_ent_s{
	struct list_head link;

	command_t *exec;
	int32_t argc;
	char *argv[NR_ARG_MAX+1];
	param_t *arg2param[NR_ARG_MAX+1];
}exec_ent_t;

#define free_exec_ent(exec_ent)	do { \
	int32_t i; \
	for (i = 0; i < (exec_ent)->argc; i++) { \
		free((exec_ent)->argv[i]); \
	} \
} while (0)

#define load_exec_ent(xsh, exec_ent) do { \
	int32_t argc = 0; \
 \
	xsh->exec = exec_ent->exec; \
	for (i = 0; i < exec_ent->argc; i++) { \
		if (!exec_ent->argv[i] && (exec_ent->arg2param[i]->type & PARAM_OPTIONAL)) \
			continue; \
		xsh->argv[argc] = exec_ent->argv[i]; \
		xsh->arg2param[argc++] = exec_ent->arg2param[i]; \
	} \
	xsh->argc = argc; \
} while (0)


#define IDTYPE_INTEGER	1
#define IDTYPE_FLOAT	2
#define IDTYPE_CHAR	3
#define IDTYPE_STRING	4

typedef struct ident_s {
	struct rb_node rb;

	struct {
		char *text;
		int32_t len;
		int32_t type;

		union {
			int32_t _int;
			double _double;
			char *_string;
		} u;
	}attr;
} ident_t;

#define init_ident(ident) do { \
	memset(ident, 0, sizeof(ident_t)); \
} while (0)

/*
 * each scope occupies a standalone symbol table.
 */
typedef struct symtab_s {
	rb_tree_t ident_tr;

	struct list_head next;
	struct list_head down;
	struct symtab_s *up;

	struct {
		/* find out an identifier entry within `symtab' or symbol tables upwards,
		 * return NULL when it fails.
		 */
		ident_t * (*find)(struct symtab_s *symtab, char *name);
		/* enter an identifier entry into `symtab', return the existed one if it
		 * duplicates.
		 */
		ident_t * (*enter)(struct symtab_s *symtab, ident_t *ident);
	}op;
}symtab_t;

extern ident_t * symtab_find(symtab_t *symtab, char *name);
extern ident_t * symtab_enter(struct symtab_s *symtab, ident_t *ident);

#define init_symtab(symtab) do { \
	rb_tree_init(&(symtab)->id_tr, \
		strcmp, \
		rb_tree_find_default, \
		rb_tree_insert_default, \
		rb_tree_erase_default, \
		(rb_treenode_del_fp)0, \
		rb_tree_finalize_default); \
 \
	INIT_LIST_HEAD(&(symtab)->next); \
	INIT_LIST_HEAD(&(symtab)->down); \
	(symtab)->up = (symtab_t *)0; \
 \
	(symtab)->find = symtab_find; \
	(symtab)->enter = symtab_enter; \
} while (0)

/**
 * xsh_t - `xsh' is a CLI interface through which an operator could perform operating & management
 *	activities. Currently, xsh supports different views of commands, emacs-like line edition,
 *	tab expansion and help infomation invokation. Although the implementaion is quite a little
 *	ugly, xsh does work very well anyway.
 *
 * @root:	root view, the first view that xsh uses after starting
 * @curr:	current working view
 * @term:	the term that xsh uses
 * @prompt:
 * @line:
 * @line_size:
 * @opts:	points to the command options if it does have one
 * @exec_pid:
 * @exec:	the resolved exact command that xsh find in its current view
 * @argc:	number of arguments
 * @argv:	arrary of arguments, including command name
 * @arg2desc:
 * @match:	list head for all possible commands when xsh try to perform tab expansion
 * @nr_match:	number of match commands
 * @histent:
 * @hbeg:
 * @hend:
 * @hist:
 */
typedef struct xsh_s{
	view_t *root;
	view_t *curr;

	symtab_t *symtab_root;
	symtab_t *symtab_curr;

	tty_t *term;
	char prompt[TTY_BUF_SIZE_MAX];
	char decor[TTY_BUF_SIZE_MAX/2];

	char line[TTY_BUF_SIZE_MAX];
	int32_t line_size;
	char *opts;
	
	int32_t nr_exec_ent;
	struct list_head exec_ent_list;

	int32_t _argc;
	command_t *exec;
	int32_t argc;
	char *argv[NR_ARG_MAX+1];
	param_t *arg2param[NR_ARG_MAX+1];

	struct list_head match;
	uint32_t nr_match;

	histent_t histent[NR_HISTENT_MAX];
	struct list_head *hbeg;
	struct list_head *hend;
	struct list_head *hist;

	uint32_t task_xsh_stk[TASK_STK_SIZE_XSH];
}xsh_t;

extern xsh_t g_vsh;


#define XSH_PANIC()     sys_panic("<XSH panic> %s : %d\n", __func__, __LINE__)


extern void vsh_init(tty_t *term);
extern void task_xsh(ULONG arg);

extern void xsh_push_hist(xsh_t *xsh, char *s);
extern int32_t xsh_resolve_cmd(xsh_t *xsh);
extern void xsh_update_view(xsh_t *xsh, view_t *view);
extern void xsh_update_decor(xsh_t *xsh, char *decor);
extern char * xsh_getopt(xsh_t *xsh);

extern void xsh_dokey_tab(void *context);
extern void xsh_dokey_help(void *context);
extern void xsh_dokey_previous(void *context);
extern void xsh_dokey_next(void *context);

extern int32_t command_cmp_ws(struct rb_node *n, void *key);
extern int32_t strprefix(char *s, char *t);
extern void print_prompt(int32_t len, xsh_t *xsh);


#endif	/* end of __VSH_H__ */
