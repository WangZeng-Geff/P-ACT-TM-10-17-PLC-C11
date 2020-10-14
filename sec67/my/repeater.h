#ifndef _REPEATER_H_
#define _REPEATER_H_

#include <stdint.h>
#include "sdk_evt.h"
#include "protocol.h"
#include "flash.h"
#include "timer.h"
#include "uart.h"
#include "RemoteRXDegree.h"
#include "comfunc.h"

//#define SMART_MODEM

#define REPEATER_VERSION         0x0102
#define REPEATER_DATE            0x191219

#define PLC_MATERIAL_CODE       "83676800202"
#define COMM_PROTOCOL_VER		    "EASTSOFT(v1.0)"
#define REPEATER_VERSION_EXT    "SSC1667-ADPT-V30B010"

//-----------------------------------------------------------
extern uint8_t taker_is_gateway;
extern int Nose_Print_Enable;
//-----------------------------------------------------------

enum LAYER_TYPE
{
    LAYER_PLC, LAYER_REPEATER, LAYER_NR
};

#pragma pack(1)
struct registerInfo
{
	uint8_t mac[MAC_LEN];
	uint8_t aid[AID_LEN];
	uint8_t len;
	uint8_t depth;
};

struct deviceInfo
{
	uint32_t magic;
    UART_BAUD baud;
	UART_TYPE_DATAMOD mod;
	uint8_t sn[SN_LEN];
	uint8_t dkey[DKEY_LEN];
	uint8_t aid[AID_LEN];
	uint8_t passwd[PW_LEN];
};

struct deviceCfgInfo
{
	uint32_t magic;
	uint8_t panid[PANID_LEN];
	uint8_t gid[AID_LEN];
	uint8_t gmac[MAC_LEN];
	uint8_t sid[SID_LEN];
};

#pragma pack()

struct Repeater
{
	uint8_t sn[SN_LEN];
	uint8_t dkey[DKEY_LEN];
    uint8_t aid[AID_LEN];
    uint8_t passwd[PW_LEN];
    uint8_t sid[SID_LEN];
    uint8_t gid[AID_LEN];
    uint8_t panid[SID_LEN];
    uint8_t mac[MAC_LEN];
    uint8_t gmac[MAC_LEN];
    UART_BAUD baud;
	UART_TYPE_DATAMOD mod;
};
extern struct Repeater repeater;

struct LocalOps
{
    const char *name;
    int cmd;
    int (*handle)(uint8_t *data, int len);
};

//---------------------------
struct DEV_DATA
{
	uint32_t magic;
	uint8_t location;
};
extern struct DEV_DATA dev_data;

struct CTRL_MEG
{
	uint8_t aid[AID_LEN];
	uint8_t mac[MAC_LEN];
};
struct RAM_DATA
{
	uint32_t silent_time;
	uint8_t search_aid[AID_LEN];
	uint8_t search_seq;
	uint8_t search_mac[MAC_LEN];
	//--------------------
	uint8_t last_ctrl_aid[AID_LEN];
	uint8_t last_ctrl_mac[MAC_LEN];
	struct CTRL_MEG ctrl_meg[MAX_CTRL_NUM];
};
extern struct RAM_DATA ram_data;

extern const char *repeater_plc_type;
extern const char *comm_protocol_ver;
extern const char *repeater_ext_ver;

void repeater_show(void);
void system_init(void);
void repeater_reset(void);

void dev_info_init(void);

const char *get_dev_ver(void);
const char *get_dev_ver1(void);
const char *get_plc_ver(void);
int get_dev_type_by_layer(enum LAYER_TYPE type, uint8_t *dev_type);
enum LAYER_TYPE get_layer_by_dev_type(const uint8_t *dev_type);
int get_dev_type(uint8_t *type);
int get_plc_type(uint8_t *type);
const char *get_soft_ver_by_layer(enum LAYER_TYPE type);

int do_set_psk(void);
int do_psk_resp(const uint8_t *dst, int state);
int do_plc_broadcast(int key_mod, const void *data, int len);
int do_plc_unicast(const uint8_t *dst, int key_mod, const void *data, int len);

void repeater_on_evt_devprop_req(const sdk_evt_devprop_req_t *req, const uint8_t *att, int len);
void repeater_on_evt_rx_plc(const sdk_evt_rx_plc_t *info, const uint8_t *data, int len);

#endif
