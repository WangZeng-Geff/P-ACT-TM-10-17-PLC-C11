#include "dev_show.h"
#include "comfunc.h"
#include "softtimer.h"
#include "sec_tick.h"
//---------------------------------------------------------------------------------------
static uint8_t sn_type_check(uint8_t data[],uint8_t sn_data[])
{
    if (is_all_xx(data, 0xFF, TYPE_LEN))
    {
        return 1;
    }
    else if (((0x0f & data[1]) == (0x0f & sn_data[1]))
            && ((0xf0 & data[3]) == (0xf0 & sn_data[3]))
            && (data[2] == sn_data[2]))
    {
        return 1;
    }
    return 0;
}
//---------------------------------------------------------------------------------------
void search_dev_timer_handle(struct soft_timer *st)
{
    uint8_t buf[0x100], body[0x100];
    uint8_t body_len = 0;
    body_len += code_body(0x0007, 0, repeater.sn, SN_LEN, body, sizeof(body));
    body_len += code_body(0x0005, 0, repeater.dkey, DKEY_LEN, body + body_len, sizeof(body) - body_len);
    body_len += code_body(0x000A, 0, repeater.passwd, PW_LEN, body + body_len, sizeof(body) - body_len);
    body_len += code_body(0x0003, 0, get_dev_ver(), strlen(get_dev_ver()), body + body_len, sizeof(body) - body_len);  
   	code_frame(repeater.aid, ram_data.search_aid, ram_data.search_seq, CMD_SHOW, body, body_len, buf, sizeof(buf));
    pr_info("[PRO-SEARCHDEV]Report Data to search_dev_timer_handle!\n");
   	do_send_frame(ram_data.search_mac, (struct SmartFrame *)buf);
}
//---------------------------------------------------------------------------------------
struct soft_timer search_dev_timer =
{
    .timer_cb = search_dev_timer_handle,
};
int search_frame_opt(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    uint8_t search_type, search_did[DID_LEN] = {0x08,0x00};
	uint16_t max_delay_time;
	struct AppFrame *app = (struct AppFrame *)pframe->data;
	uint8_t gid_len = get_bits(app->data[0], 0, 5) + 1;
	struct Body *body = (struct Body *)(app->data + gid_len);
    if(!(is_all_xx(pframe->taid, 0xFF, AID_LEN)) || memcmp(body->did, search_did, DID_LEN)) 
    {
    	return -1;
    } 
    if((ram_data.silent_time && (time_after_eq(ram_data.silent_time, sec_jiffies))) || (!is_all_xx(repeater.panid, 0x00, PANID_LEN))) 
    {
    	return -2;
    }   
    if (!is_gid_equal(app->data, repeater.sid))
    {
    	return -3;
    }
    search_type = body->data[0];
    if(search_type || !sn_type_check(&body->data[1 + TIME_LEN], repeater.sn))
    {
    	return -4;
    }
	max_delay_time = get_le_val(&body->data[1], TIME_LEN);
    if(!max_delay_time) max_delay_time = 10;
    srand(sec_jiffies + get_le_val(repeater.aid, AID_LEN));
	if (!search_dev_timer.expires || !time_after_eq(search_dev_timer.expires, sec_jiffies))
    {
		search_dev_timer.expires = sec_jiffies + ((rand() % (max_delay_time))) * SEC_HZ;
		soft_timer_add(&search_dev_timer);
    }
	else
	{
		search_dev_timer.expires = sec_jiffies + ((rand() % (max_delay_time))) * SEC_HZ;
	}
    ram_data.search_seq = pframe->seq | 0x80;
    memcpy(ram_data.search_mac, info->src, MAC_LEN);
    memcpy(ram_data.search_aid, pframe->said, AID_LEN);
    
    return 0;
}
