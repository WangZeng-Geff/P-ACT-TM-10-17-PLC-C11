#include "include.h"
#include "comfunc.h"
#include "repeater.h"
#include "sec_printf.h"
#include "sdk_svc.h"
#include "cmd.h"
#include "flash.h"
#include "esuart.h"
#include "repeater.h"

//---------------------------------------------------------------------------------------
static param_t cmd_dev_param_tab[] = {
	{PARAM_ARG|PARAM_TOGGLE, "", "show|reset|get|set" },
    {PARAM_FLG|PARAM_SUB|PARAM_OPTIONAL, "prlvl", "", "get"},
    {PARAM_OPT|PARAM_INTEGER|PARAM_SUB|PARAM_OPTIONAL, "prlvl", "", "set"},
	{PARAM_OPT|PARAM_INTEGER|PARAM_SUB|PARAM_OPTIONAL, "panid", "", "set"},
    {PARAM_FLG|PARAM_SUB|PARAM_OPTIONAL, "info", "", "get"},
    {PARAM_OPT|PARAM_STRING|PARAM_SUB|PARAM_OPTIONAL, "info", "", "set"},
    {PARAM_OPT|PARAM_STRING|PARAM_SUB|PARAM_OPTIONAL, "test", "", "set"},
	PARAM_EMPTY
};
static cmd_desc_t cmd_dev_desc = CMD_DESC_ENTRY(
							"", cmd_dev_param_tab
							);

static void docmd_dev(command_t *cmd, xsh_t *xsh)
{
    if (!strcmp(xsh->argv[1], "show"))
    {
        repeater_show();
    } 
    else if (!strcmp(xsh->argv[1], "reset"))
    {
        const char *psk = "eastsoft";
        sdk_svc_psk_t info;
        strcpy((char*)info.psk, psk); 
        info.len = strlen(psk);
        sdk_svc_set_psk(&info);
        repeater_reset();
    }
    else if (!strcmp(xsh->argv[1], "get")) 
    {
        if (strstr(xsh->argv[2], "prlvl")) 
        {
            esprintf("Sec print level: %d\n", sec_get_pr_lvl());
        }
		else if (strstr(xsh->argv[2], "info"))
        {
            struct deviceInfo info;
        	memcpy(&info, (uint8_t*)(DEVICE_INFO_ADDR), sizeof(info));
            if (info.magic != DEVICE_INFO_MAGIC)
            {
                pr_emerg("devinfo invalid, need to be written!!!\n");
            }
            else
            {
                print_debug_array(KERN_EMERG, &info.sn,
                                  sizeof(info)-offset_of(struct deviceInfo, sn));
            }
        }
    }
    else if (!strcmp(xsh->argv[1], "set")) 
    {
        if (strstr(xsh->argv[2], "prlvl")) 
        {
            if ('8' == *(strchr(xsh->argv[2], '=') + 1)) {//新加过滤干扰选项
                Nose_Print_Enable = 1;
            }else if('9' == *(strchr(xsh->argv[2], '=') + 1)){
                Nose_Print_Enable = 0;
            }else{
                sec_set_pr_lvl(strtoul(strchr(xsh->argv[2], '=') + 1, NULL, 10));
            }
        }
        else if (strstr(xsh->argv[2], "info"))
        {
            struct deviceInfo info;
            int need = sizeof(info) - offset_of(struct deviceInfo, sn);

            int len = str2arr(strchr(xsh->argv[2], '=') + 1, info.sn, need);
            if (len != need) {
                pr_emerg("devinfo len[%d] error, should be [%d]!\n", len, need);
                return;
            }
            info.magic = DEVICE_INFO_MAGIC;
			info.baud  = UART_BAUD_9600;//UART_BAUD_19200
			info.mod = UART_DataMode_8;
            flash_user_info_write(DEVICE_INFO_ADDR, (uint8_t *)&info, sizeof(info));
            dev_info_init();
        }
		else if (strstr(xsh->argv[2], "panid"))
        {
			uint8_t panid[PANID_LEN];
        	int len = str2arr(strchr(xsh->argv[2], '=') + 1, panid, PANID_LEN);
        	if (len != PANID_LEN)
        	{
        	    pr_emerg("panid len[%d] error, should be 2!\n", len);
        		return;
        	}
    		memcpy(repeater.panid, panid, PANID_LEN);
			do_set_psk();
    		struct deviceCfgInfo cfg;
    		memcpy(&cfg, (uint8_t *)DEVICE_CFG_ADDR, sizeof(cfg));
    		memcpy(cfg.panid, repeater.panid, PANID_LEN);
			cfg.magic = DEVICE_CFG_MAGIC;
    		flash_user_info_write(DEVICE_CFG_ADDR, (uint8_t *)&cfg, sizeof(cfg));
        }
		else if (strstr(xsh->argv[2], "test"))
        {
        	uint8_t info[BUF_LEN];
        	int len = str2arr(strchr(xsh->argv[2], '=') + 1, info, BUF_LEN);
        	esuart_send_data(UART_CHN_DM, info, len);
        }
    }
}
//---------------------------------------------------------------------------------------
static view_t view_dev[] =
{
	VIEWTAB_ENTRY(dev,   "[dev] "),
};

static command_t cmd_dev[] =
{
	CMDTAB_ENTRY(dev, root,  dev,  docmd_dev, &cmd_dev_desc),
};

void sec_cmd_init(void)
{
    cmd_init(view_dev, array_size(view_dev), cmd_dev, array_size(cmd_dev));
}
//---------------------------------------------------------------------------------------
