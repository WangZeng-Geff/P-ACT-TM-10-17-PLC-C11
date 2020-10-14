/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		sdk_svc.h
Description:	The service interface for high layer.
Author:			eastsoft
Version:		v1.0
Date:			2016/01/25
History:
*/
/************************************************************************************************/

#ifndef _SDK_SVC_H_
#define _SDK_SVC_H_

#include "sdk.h"

typedef struct _sdk_svc_ver_t								/* 版本信息. */
{
	u16					manu;								/* 制造商标志,东软:"ES"(0x4553). */
	u8					chl;								/* 信道类型. */
	u8					mod;								/* 模块类型. */
	u16					config;								/* 配置信息. */
	u16					ver;								/* 版本号. */
	u32					date;								/* 版本时间. */
} sdk_svc_ver_t;

typedef struct _sdk_svc_mac_t								/* MAC地址. */
{
	u8					mac[SDKaMACLen];					/* MAC地址. */
} sdk_svc_mac_t;

typedef struct _sdk_svc_gpid_t								/* 组地址. */
{
	u8					cnt;								/* 组地址数量. */
	u16					gpid[SDKaMaxGpidNum];				/* 组播地址队列. */
} sdk_svc_gpid_t;

typedef struct _sdk_svc_psk_t								/* 密钥. */
{
	u8					psk[64];							/* 密钥. */
	u8					len;								/* 密钥长度. */
} sdk_svc_psk_t;

typedef struct _sdk_svc_uni_t								/* 单播参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_prio_t			prio;								/* 优先级. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	sdk_key_mod_t		key_mod;							/* 密钥模式. */
	u16					dst_gpid;							/* 组播地址. */
	u8					dst[SDKaMACLen];					/* 目的MAC. */
} sdk_svc_uni_t;

typedef struct _sdk_svc_brd_t								/* 广播参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_prio_t			prio;								/* 优先级. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	sdk_key_mod_t		key_mod;							/* 密钥模式. */
	u16					dst_gpid;							/* 目的组播地址 */
	u8					radius;								/* 广播半径. */
} sdk_svc_brd_t;

typedef struct _sdk_svc_sch_start_t							/* 开始设备搜索参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_sch_rule_t		rule;								/* 设备搜索规则. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					dst_gpid;							/* 组播地址. */
	u8					radius;								/* 设备搜索半径. */
} sdk_svc_sch_start_t;

typedef struct _sdk_svc_devprop_resp_t						/* 回复设备搜索属性参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_sch_state_t		state;								/* 是否参与设备搜索. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					src_gpid;							/* 组播地址. */
	u8					src[SDKaMACLen];					/* 发起设备搜索节点地址. */
	u8					taskid;								/* 本次设备搜索任务号. */
} sdk_svc_devprop_resp_t;

typedef struct _sdk_svc_rmtpsk_req_t						/* 远程设置口令请求参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_prio_t			prio;								/* 请求的优先级. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					dst_gpid;							/* 组播地址. */
	u8					dst[SDKaMACLen];					/* 目的MAC地址. */
	u8					*old_key;							/* 目的节点的旧口令. */
	u8					old_len;							/* 旧口令长度. */
	u8					*new_key;							/* 新口令. */
	u8					new_len;							/* 新口令长度. */
} sdk_svc_rmtpsk_req_t;

typedef struct _sdk_svc_rmtpsk_resp_t						/* 远程设置口令回复参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_prio_t			prio;								/* 回复的优先级. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					dst_gpid;							/* 组播地址. */
	u8					dst[SDKaMACLen];					/* 发出请求的节点的MAC地址. */
	sdk_rmtpsk_state_t	state;								/* 是否同意修改口令. */
} sdk_svc_rmtpsk_resp_t;

typedef struct _sdk_svc_rmtsgid_req_t						/* 远程修改组播地址请求参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					dst_gpid;							/* 组播地址. */
	u8					dst[SDKaMACLen];					/* 目的MAC地址. */
	u8					gidcnt;								/* 组播地址 */
	u16					gid[SDKaMaxGpidNum];				/* 组播地址列表 */
} sdk_svc_rmtsgid_req_t;

typedef struct _sdk_svc_rmtsgid_resp_t						/* 远程修改组播地址回复参数. */
{
	sdk_addr_mod_t		src_mod;							/* 源地址模式. */
	sdk_addr_mod_t		dst_mod;							/* 目的地址模式. */
	sdk_multi_t			multi;								/* 组播地址标志. */
	u16					dst_gpid;							/* 组播地址. */
	u8					dst[SDKaMACLen];					/* 发出请求的节点的MAC地址. */
	sdk_rmtsgid_state_t	state;								/* 是否同意修改组播地址. */
} sdk_svc_rmtsgid_resp_t;

typedef struct _sdk_svc_ge_init_t							/* GE初始化参数. */
{
	sdk_ge_bridge_t		bridge;								/* 是否初始化为网桥, 只对64有效. */
	sdk_ge_dhcp_t		dhcp;								/* 初始化后是否直接开启DHCP. 64,63均可. */
} sdk_svc_ge_init_t;

typedef struct _sdk_svc_ge_info_t							/* 网口信息参数. */
{
	u32					ip;									/* IP地址. */
	u32					mask;								/* 子网掩码. */
	u32					gw;									/* 网关. */
	u32					fri_dns;							/* 首选DNS服务器. */
	u32					sec_dns;							/* 备用DNS服务器. */
} sdk_svc_ge_info_t;

typedef struct _sdk_svc_que_ge_t							/* 读取连接参数. */
{
	u8					num;								/* 读取到的连接数量. */
	sdk_link_state_t	link[SDKaMaxLinkNum];				/* 连接信息. */
} sdk_svc_que_ge_t;

typedef struct _sdk_svc_tst_sfo_t							/* 频偏通知测试参数. */
{
	u8					dst[SDKaMACLen];					/* 传入参数, 目的MAC地址. */
} sdk_svc_tst_sfo_t;

typedef struct _sdk_svc_tst_get_sfo_t						/* 读取频偏通知测试参数. */
{
	u8					dst[SDKaMACLen];					/* 传入参数, 目的MAC地址. */
	u16					sfo;								/* 传出参数, 频偏值. */
} sdk_svc_tst_get_sfo_t;

typedef struct _sdk_svc_tst_req_t							/* 发送测试数据请求. */
{
	u8					dst[SDKaMACLen];					/* 传入, 目的地址. */
	sdk_tmi_t			tmi;								/* 传入, TMI. */
	sdk_frm_t			frm;								/* 传入, FRM 信息. */
} sdk_svc_tst_req_t;

typedef struct _sdk_svc_tst_gain_t							/* 读取gain配置. */
{
	s8					rgain;								/* 传出, 接收功率. */
	s8					tgain;								/* 传出, 发送功率. */
} sdk_svc_tst_gain_t;

typedef struct _sdk_svc_tst_getnb_t
{
	u32					amt;								/* 传入, 需要获取的MAC地址最大数量, 最大值为500. */
															/* 传出, 实际获得的MAC地址数量, 小于等于传入值. */
	u8					*mac;								/* 传入, 存放内存, size >= amt * 6. */
															/* 传出, mac地址序列. */
} sdk_svc_tst_getnb_t;

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_reset
	Description	:	重启
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
extern void sdk_svc_reset(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_ver
	Description	:	读取版本信息
	Input		:	info - 存储版本信息的内存
	Output		:	info - 读取到的版本信息
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_ver(sdk_svc_ver_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_mac
	Description	:	读取MAC地址
	Input		:	info - 存储MAC地址的内存
	Output		:	info - 读取到的MAC地址
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_mac(sdk_svc_mac_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_gpid
	Description	:	读取组地址
	Input		:	info - 存储组地址的内存
	Output		:	info - 读取到的组地址
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_gpid(sdk_svc_gpid_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_gpid
	Description	:	设置组地址
	Input		:	info - 需要设置的组地址
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_gpid(sdk_svc_gpid_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_psk
	Description	:	读取本地口令
	Input		:	info - 存储本地口令的内存
	Output		:	info - 读取到的本地口令
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_psk(sdk_svc_psk_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_psk
	Description	:	设置本地口令
	Input		:	info - 需要设置的本地口令
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_psk(sdk_svc_psk_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_hiepsk
	Description	:	读取本地主干口令
	Input		:	info - 存储本地主干口令的内存
	Output		:	info - 读取到的本地主干口令
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_hiepsk(sdk_svc_psk_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_hiepsk
	Description	:	设置本地主干口令
	Input		:	info - 需要设置的本地主干口令
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_hiepsk(sdk_svc_psk_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_mac
	Description	:	设置MAC地址
	Input		:	info - 需要设置的MAC地址
	Output		:	none
	Return		:	状态码
	Notes		:	MAC地址写入FLASH, 有工装后该指令删除
					64设置MAC地址后需重启
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_mac(sdk_svc_mac_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_unicast
	Description	:	电力线单播
	Input		:	info - 单播参数
					data - 单播数据
					len  - 数据长度
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_unicast(sdk_svc_uni_t *info, u8 *data, u32 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_brodcast
	Description	:	电力线广播
	Input		:	info - 广播参数
					data - 广播数据
					len  - 数据长度
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_brodcast(sdk_svc_brd_t *info, u8 *data, u32 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_sch_start
	Description	:	开始设备搜索
	Input		:	info - 设备搜索参数
					att  - 发起设备搜索中的属性数据
					len  - 属性长度
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_sch_start(sdk_svc_sch_start_t *info, u8 *att, u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_sch_stop
	Description	:	停止设备搜索
	Input		:	none
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_sch_stop(void);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_devprop_resp
	Description	:	回复设备搜索属性
	Input		:	info - 回复设备搜索属性参数
					att  - 回复设备搜索属性数据
					len  - 属性长度
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_devprop_resp(sdk_svc_devprop_resp_t *info, u8 *att, u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmtpsk_req
	Description	:	远程设置口令请求
	Input		:	info - 请求参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmtpsk_req(sdk_svc_rmtpsk_req_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmtpsk_resp
	Description	:	远程设置口令回复
	Input		:	info - 回复参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmtpsk_resp(sdk_svc_rmtpsk_resp_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmthiepsk_req
	Description	:	远程设置主干口令请求
	Input		:	info - 请求参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmthiepsk_req(sdk_svc_rmtpsk_req_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmthiepsk_resp
	Description	:	远程设置主干口令回复
	Input		:	info - 回复参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmthiepsk_resp(sdk_svc_rmtpsk_resp_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmtsgid_req
	Description	:	远程修改组播地址请求
	Input		:	info - 请求参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmtsgid_req(sdk_svc_rmtsgid_req_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_rmtsgid_resp
	Description	:	远程修改组播地址回复
	Input		:	info - 回复参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_rmtsgid_resp(sdk_svc_rmtsgid_resp_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_tone
	Description	:	设置载波参数
	Input		:	tone  - 子载波
					tgain - 发送功率
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_tone(sdk_tone_t tone, int tgain);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_en_ge
	Description	:	使能TCP/IP模式
	Input		:	info - 初始化参数
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_en_ge(sdk_svc_ge_init_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_get_ge
	Description	:	读取网口信息
	Input		:	info - 存储网口信息的内存
	Output		:	info - 都去到的网口信息
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_get_ge(sdk_svc_ge_info_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_ge
	Description	:	设置网口信息
	Input		:	info - 将要设置的网口信息
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_ge(sdk_svc_ge_info_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_start_dns
	Description	:	开始DNS
	Input		:	host - 主机名
					len  - 主机名长度
	Output		:	none
	Return		:	状态码
	Notes		:	同一时间只处理一条DNS解析
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_start_dns(u8 *host, u8 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_que_ge
	Description	:	读取连接状态
	Input		:	pair - 套接字对
					info - 存储连接状态的内存
	Output		:	info - 读取到的连接状态
	Return		:	状态码
	Notes		:	套接字对全0, 查询所有连接; 目的为0,
					匹配源套接字; 源为0, 匹配目的套接字
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_que_ge(sdk_skt_pair_t *pair, sdk_svc_que_ge_t *info);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_link_ge
	Description	:	建立TCP/UDP连接
	Input		:	type - 连接类型
					pair - 套接字对
	Output		:	none
	Return		:	状态码
	Notes		:	若本地端口为0, 则从49153向上分配本地端口号
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_link_ge(sdk_link_type_t type, sdk_skt_pair_t *pair);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_del_ge
	Description	:	删除连接
	Input		:	pair - 套接字对
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_del_ge(sdk_skt_pair_t *pair);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_tx_ge
	Description	:	发送TCP/UDP数据
	Input		:	pair - 套接字对
					data - 发送的数据
					len  - 数据长度
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_tx_ge(sdk_skt_pair_t *pair, u8 *data, u32 len);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_set_updflag
	Description	:	升级固件
	Input		:	tp - 升级包类型
	Output		:	none
	Return		:	状态码
	Notes		:	
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_set_updflag(sdk_upd_type_t tp);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_tst
	Description	:	测试指令
	Input		:	cmd  - 测试命令字
					para - 命令参数	/sdk_svc_tst_sfo_t/sdk_svc_tst_get_sfo_t/sdk_svc_tst_req_t
									/sdk_svc_tst_gain_t
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_tst(sdk_tst_cmd_t cmd, void *para);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_aes_enc
	Description	:	aes加密接口
	Input		:	cmd  - None
					din - 加密接口输入的明文
					inlen - 明文数据长度
					dout - 加密之后输出的密文
					key_enc - 加密秘钥
	Output		:	none
	Return		:	状态码
	Notes		:	1.加密秘钥和解密秘钥为同一秘钥
					2.明文是按照16字节为单位加密，不为16字节整数倍的，截断解密
					3.该函数内有有操作信号量，所以请不要在中断服务子程序和timer的回调函数中使用
					4.加解密秘钥的长度为16字节
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_aes_enc(u8 *din, const u32 inlen, u8 *dout, u8 *key_enc);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_aes_dec
	Description	:	aes加密接口
	Input		:	cmd  - None
					din - 解密接口输入的密文
					inlen - 密文数据长度
					dout - 解密之后输出的明文
					key_dec - 解密秘钥
	Output		:	none
	Return		:	状态码
	Notes		:	1.加密秘钥和解密秘钥为同一秘钥
					2.密文是按照16字节为单位解密，不为16字节整数倍的，截断解密
					3.该函数内有有操作信号量，所以请不要在中断服务子程序和timer的回调函数中使用
					4.加解密秘钥的长度为16字节
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_aes_dec(u8 *din, const u32 inlen, u8 *dout, u8 *key_dec);

/*****************************************************************************************************/
/*
	Function	:	sdk_svc_md5
	Description	:	测试指令
	Input		:	cmd  - None
					data - 生成MD5的加密数据
					dlen - 数据长度
					md5 - MD5结果的出参，长度不得小于16Byte
	Output		:	none
	Return		:	状态码
	Notes		:	none
*/
/*****************************************************************************************************/
extern sdk_err_t sdk_svc_md5(const u8 *data, const u32 dlen, u8 *md5);

#endif
