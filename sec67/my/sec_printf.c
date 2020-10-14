#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "timer.h"
#include "cmd.h"
#include "comfunc.h"
#include "sec_printf.h"
#include "repeater.h"

//---------------------------------------------------------------------------------------
static char pr_cache[0x1000];
static int pr_lvl = 0;
//---------------------------------------------------------------------------------------
void sec_printf(const char *fmt, ...)
{
    int lvl = 0;
    char *text = pr_cache;
	va_list args;

	va_start(args, fmt);

    vsnprintf(pr_cache, sizeof(pr_cache), fmt, args);

	if (text[0] == KERN_SOH_ASCII && text[1]) 
    {
        switch (text[1])
        {
        case '0' ... '7':
            lvl = text[1] - '0';
            text += 2;
            break;
        }
    }
    if (lvl <= pr_lvl) 
    {
        esprintf(text);
    }

	va_end(args);
}
//---------------------------------------------------------------------------------------
void sec_set_pr_lvl(int lvl)
{
    if (lvl >= 0 && lvl <= 7) 
    {
        pr_lvl = lvl;
    }
}
int sec_get_pr_lvl(void)
{
    return pr_lvl;
}
//---------------------------------------------------------------------------------------
