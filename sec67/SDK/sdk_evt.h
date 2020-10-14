/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		sdk_evt.h
Description:	The event interface for high layer.
Author:			eastsoft
Version:		v1.0
Date:			2016/01/25
History:
*/
/************************************************************************************************/

#ifndef _SDK_EVT_H_
#define _SDK_EVT_H_

#include "sdk.h"

typedef struct _sdk_evt_tx_state_t							/* 电力线发送异常状态上报参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					dst[SDKaMACLen];					/* 目的MAC地址. */
	sdk_tx_state_t		state;								/* 错误状态. */
} sdk_evt_tx_state_t;

typedef struct _sdk_evt_rx_plc_t							/* 电力线接收数据参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					depth;								/* 深度. */
	sdk_key_mod_t		key_mod;							/* 密钥模式. */
	sdk_multi_t			multi;								/* 组播标志. */
	u16					src_gpid;							/* 源节点组地址. */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
} sdk_evt_rx_plc_t;

typedef struct _sdk_evt_devprop_req_t						/* 申请设备搜索属性参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_multi_t			multi;								/* 组播标志. */
	u16					dst_gpid;							/* 目的节点组地址. */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
	u8					taskid;								/* 本次搜索任务号. */
} sdk_evt_devprop_req_t;

typedef struct _sdk_evt_node_info_t							/* 节点信息参数. */
{
	u8					*mac;								/* 被搜节点MAC地址. */
	u8					depth;								/* 被搜节点深度. */
	sdk_net_state_t		net;								/* 被搜节点网络状态. */
	sdk_addr_mod_t		mod;								/* 被搜节点地址模式. */
	sdk_multi_t			multi;								/* 组播标志. */
	u16					gpid;								/* 源节点组地址. */
	u8					*att;								/* 被搜节点上报的属性数据. */
	u8					len;								/* 属性数据长度. */
} sdk_evt_node_info_t;

typedef struct _sdk_evt_sch_rpt_t							/* 上报搜索设备参数. */
{
	u8					num;								/* 搜索到的节点数量. */
	sdk_evt_node_info_t	info[SDKaSearMaxRptclt];			/* 被搜节点信息. */
} sdk_evt_sch_rpt_t;

typedef struct _sdk_evt_rmtpsk_req_t						/* 上报搜索设备参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					depth;								/* 深度. */
	sdk_multi_t			multi;								/* 组播地址标志 */
	u16					gid;								/* 组播地址 */
	sdk_rmtpsk_dec_t	state;								/* 解密状态. */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
} sdk_evt_rmtpsk_req_t;

typedef struct _sdk_evt_rmtpsk_resp_t						/* 远程设置口令回复上报参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					depth;								/* 深度. */
	sdk_multi_t			multi;								/* 组播地址标志 */
	u16					gid;								/* 组播地址 */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
	sdk_rmtpsk_state_t	state;								/* 是否同意修改口令. */
} sdk_evt_rmtpsk_resp_t;

typedef struct _sdk_evt_rmtsgid_req_t 						/* 远程修改组播地址请求上报参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					depth;								/* 深度 */
	sdk_multi_t			multi;								/* 组播地址标志 */
	u16					gid;								/* 组播地址 */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
	sdk_rmtsgid_state_t	state;								/* 组播地址是否有效 */
} sdk_evt_rmtsgid_req_t;

typedef struct _sdk_evt_rmtsgid_resp_t						/* 远程修改组播地址回复上报参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	u8					depth;								/* 深度. */
	sdk_multi_t			multi;								/* 组播地址标志 */
	u16					gid;								/* 组播地址 */
	u8					src[SDKaMACLen];					/* 源节点MAC地址. */
	sdk_rmtsgid_state_t	state;								/* 是否同意修改口令. */
} sdk_evt_rmtsgid_resp_t;

typedef struct _sdk_evt_tst_resp_t							/* 发送测试数据应答. */
{
	u8					src[SDKaMACLen];					/* 源地址. */
	sdk_tmi_t			tmi;								/* TMI. */
	sdk_frm_t			frm;								/* FRM 信息. */
	s8					rgain;								/* 接收功率. */
} sdk_evt_tst_resp_t;

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_state
	Description	:	电力线发送异常状态上报
	Input		:	state - 上报参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_tx_state(const sdk_evt_tx_state_t *state);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_plc
	Description	:	电力线接收数据上报
	Input		:	info - 接收参数
					data - 接收数据
					len  - 数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rx_plc(const sdk_evt_rx_plc_t *info, const u8 *data, const u32 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_devprop_req
	Description	:	申请设备搜索属性
	Input		:	req - 申请设备搜索属性参数
					att - 申请设备搜索属性数据
					len - 申请设备搜索属性数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_devprop_req(const sdk_evt_devprop_req_t *req, const u8 *att, const u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_sch_rpt
	Description	:	上报搜索设备
	Input		:	resp - 上报搜索设备参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_sch_rpt(const sdk_evt_sch_rpt_t *resp);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_req
	Description	:	远程设置口令请求上报
	Input		:	req - 请求参数
					key - 新口令
					len - 新口令长度
	Output		:	none
	Return		:	none
	Notes		:	只有解密状态为SDK_DEC_NEW时, key和len有效
*/
/*****************************************************************************************************/
extern void sdk_evt_rmtpsk_req(const sdk_evt_rmtpsk_req_t *req, const u8 *key, const u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_resp
	Description	:	远程设置口令回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rmtpsk_resp(const sdk_evt_rmtpsk_resp_t *resp);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmthiepsk_req
	Description	:	远程设置主干口令请求上报
	Input		:	req - 请求参数
					key - 新口令
					len - 新口令长度
	Output		:	none
	Return		:	none
	Notes		:	只有解密状态为SDK_DEC_NEW时, key和len有效
*/
/*****************************************************************************************************/
extern void sdk_evt_rmthiepsk_req(const sdk_evt_rmtpsk_req_t *req, const u8 *key, const u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmthiepsk_resp
	Description	:	远程设置口令回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rmthiepsk_resp(const sdk_evt_rmtpsk_resp_t *resp);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_resp
	Description	:	远程修改组播地址请求上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rmtsgid_req (const sdk_evt_rmtsgid_req_t *req, const u8 cnt, const u8 *gid);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_resp
	Description	:	远程修改组播地址回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rmtsgid_resp(const sdk_evt_rmtsgid_resp_t *resp);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_dns
	Description	:	DNS解析结果上报
	Input		:	name - 主机名
					len  - 主机名长度
					ip   - 主机IP
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_dns(const u8 *name, const u8 len, const u32 ip);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_skt_state
	Description	:	连接状态上报
	Input		:	sta - 连接状态
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_skt_state(const sdk_link_state_t *sta);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ge
	Description	:	网口接收数据上报
	Input		:	pair - 套接字对
					data - 接收数据
					len  - 接收数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rx_ge(const sdk_skt_pair_t *pair, const u8 *data, const u32 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_ledon
	Description	:	PLC TX开灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_tx_ledon(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_ledoff
	Description	:	PLC TX关灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_tx_ledoff(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ledon
	Description	:	PLC RX开灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rx_ledon(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ledon
	Description	:	PLC RX关灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_rx_ledoff(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tst_resp
	Description	:	发送测试数据接收
	Input		:	resp - 接收参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_tst_resp(const sdk_evt_tst_resp_t *resp);

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_getsec_verdate
	Description	:	获取二次开发上层的版本号和版本日期
	Input		:	version:版本号：2字节小端
					verdate:版本日期：年月日组成的3字节版本日期，小端
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_evt_getsec_ver(u8 *version, u8 *verdate);


#endif
