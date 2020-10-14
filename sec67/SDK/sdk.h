/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		sdk.h
Description:	The general interface for high layer.
Author:			eastsoft
Version:		v1.0
Date:			2016/01/25
History:
*/
/************************************************************************************************/

#ifndef _SDK_H_
#define _SDK_H_

#define SDKaMACLen			(6)						/* MAC地址长度，字节 */
#define SDKaSearMaxRptclt	(5)						/* 设备搜索一次回复最多上报节点个数 */
#define SDKaMaxLinkNum		(10)					/* 一次最多支持的链接数量 */
#define SDKaMaxGpidNum		(3)						/* 支持的最大组节点个数 */
#define SDKaMaxPskLen		(64)					/* 最大网络层口令长度 */
#define SDKaMinPskLen		(8)						/* 最小网络层口令长度 */
#define SDKaMaxENCLen		(2048)					/* 最大的加解密和MD5长度 */
#define SDKaMinENCLen		(6)						/* 最小的加解密和MD5长度 */

typedef void * func_t;
typedef void ** func_ptr_t;
typedef void ** func_tbl_t;
typedef void *** funcptr_tbl_t;

typedef enum _sdk_err_t										/* 执行状态码. */
{
	SDK_ERR_OK			= 0,								/* 正常返回状态. */
	SDK_ERR_PARA		= -1,								/* 返回参数错误. */
	SDK_ERR_BUF			= -2,								/* 返回内存不足. */
	SDK_ERR_STATE		= -3,								/* 返回状态错误. */
	SDK_ERR_LEN			= -4,								/* 返回长度错误. */
	SDK_ERR_FLASH		= -5,								/* 返写Flash错误. */
	SDK_ERR_ENC			= -6,								/* 加密错误 */
	SDK_ERR_DEC			= -7,								/* 解密错误 */

	SDK_ERR_DEF,											/* 无效状态. */
} sdk_err_t;

typedef enum _sdk_addr_mod_t								/* 地址模式. */
{
	SDK_MOD_ANY			= 0x00,								/* 不区分设备 */
	SDK_MOD_GW			= 0x01,								/* 网关设备 */
	SDK_MOD_CLOUD		= 0x02,								/* 云端设备 */
	SDK_MOD_6364		= 0x03,								/* SSC1663/64设备 */
	SDK_MOD_42			= 0x04,								/* SSC1642设备 */
	SDK_MOD_BLE			= 0x05,								/* BLE设备 */

	SDK_MOD_DEF,											/* 无效地址模式. */
} sdk_addr_mod_t;

typedef enum _sdk_prio_t									/* 优先级. */
{
	SDK_PRIO_LOW		= 0x00,								/* 低优先级. */
	SDK_PRIO_HIGH		= 0x01,								/* 高优先级. */

	SDK_PRIO_DEF,											/* 无效优先级. */
} sdk_prio_t;

typedef enum _sdk_key_mod_t									/* 密钥模式. */
{
	SDK_KEY_INIT		= 0x00,								/* 初始密钥. */
	SDK_KEY_USER		= 0x01,								/* 用户密钥. */

	SDK_KEY_DEF,											/* 无效密钥状态. */
} sdk_key_mod_t;

typedef enum _sdk_multi_t									/* 组播状态. */
{
	SDK_MULTI_DIS		= 0x00,								/* 非组播. */
	SDK_MULTI_EN		= 0x01,								/* 组播. */

	SDK_MULTI_DEF,											/* 无效组播状态. */
} sdk_multi_t;

typedef enum _sdk_sch_rule_t								/* 设备搜索规则. */
{
	SDK_SCH_ANY			= 0x00,								/* 搜索所有设备. */
	SDK_SCH_SAMENET		= 0x01,								/* 搜索同网络设备. */
	SDK_SCH_NEW			= 0x02,								/* 搜索新增设备. */

	SDK_SCH_DEF,											/* 无效搜索规则. */
} sdk_sch_rule_t;

typedef enum _sdk_sch_state_t								/* 是否参与设备搜索. */
{
	SDK_SCH_ABSENT		= 0x00,								/* 不参与设备搜索. */
	SDK_SCH_PARTICIPATE	= 0x01,								/* 参与设备搜索. */

	SDK_SCH_STATE_DEF,										/* 无效状态. */
} sdk_sch_state_t;

typedef enum _sdk_rmtpsk_state_t							/* 远程设置口令回复状态. */
{
	SDK_RMTPSK_REFUSE	= 0x00,								/* 拒绝修改. */
	SDK_RMTPSK_AGREE	= 0x01,								/* 同意修改. */

	SDK_RMTPSK_DEF,											/* 无效状态. */
} sdk_rmtpsk_state_t;

typedef enum _sdk_rmtsgid_state_t							/* 远程修改组播地址回复状态. */
{
	SDK_RMTSGID_REFUSE	= 0x00,								/* 拒绝修改. */
	SDK_RMTSGID_AGREE	= 0x01,								/* 同意修改. */

	SDK_RMTSGID_DEF,										/* 无效状态. */
} sdk_rmtsgid_state_t;

typedef enum _sdk_tone_t									/* 子载波参数. */
{
	SDK_TONE_131		= 0x00,								/* 131子载波. */
	SDK_TONE_411		= 0x01,								/* 411子载波. */

	SDK_TONE_DEF,											/* 无效状态. */
} sdk_tone_t;

typedef enum _sdk_ge_bridge_t								/* 64初始化为网桥. */
{
	SDK_BRIDGE_OFF		= 0x00,								/* 初始化为普通设备. */
	SDK_BRIDGE_ON		= 0x01,								/* 初始化为网桥. */

	SDK_BRIDGE_DEF,											/* 无效状态. */
} sdk_ge_bridge_t;

typedef enum _sdk_ge_dhcp_t									/* DHCP使能开启状态. */
{
	SDK_DHCP_INIT_OFF	= 0x00,								/* 使能TCP/IP后不开启DCHP. */
	SDK_DHCP_INIT_ON	= 0x01,								/* 使能TCP/IP后开启DCHP. */

	SDK_DHCP_INIT_DEF,										/* 无效状态. */
} sdk_ge_dhcp_t;

typedef enum _sdk_rmtpsk_dec_t								/* 远程设置口令解密状态. */
{
	SDK_DEC_FAIL		= 0x00,								/* 无法解密. */
	SDK_DEC_NEW			= 0x01,								/* 解密新口令, 本节点此时使用旧口令. */
	SDK_DEC_OLD			= 0x02,								/* 解密旧口令, 本节点此时使用新口令. */

	SDK_DEC_DEF,											/* 无效状态. */
} sdk_rmtpsk_dec_t;

typedef enum _sdk_link_sta_t								/* 连接状态. */
{
	SDK_LINK_NONE		= 0x00,								/* 无连接. */
	SDK_LINK_LINKING	= 0x01,								/* 连接中. */
	SDK_LINK_CANT		= 0x02,								/* 无法连接. */
	SDK_LINK_LOSE		= 0x03,								/* 连接丢失. */
	SDK_LINK_COMP		= 0x04,								/* 连接建立. */

	SDK_LINK_STA_DEF,										/* 无效状态. */
} sdk_link_sta_t;

typedef enum _sdk_link_type_t								/* 连接类型. */
{
	SDK_LINK_UDP		= 0x00,								/* UDP连接. */
	SDK_LINK_TCP		= 0x01,								/* TCP连接. */

	SDK_LINK_TYPE_DEF,										/* 无效状态. */
} sdk_link_type_t;

typedef enum _sdk_tx_state_t								/* 电力线发送状态. */
{
	SDK_TX_DATAF		= 0x00,								/* 无有效路径 */
	SDK_TX_CSMA			= 0x01,								/* CSMA失败 */
	SDK_TX_DATERR		= 0x02,								/* 通信失败回复错误帧 */

	SDK_TX_DEF,												/* 无效状态. */
} sdk_tx_state_t;

typedef enum _sdk_net_state_t								/* 网络状态. */
{
	SDK_NET_NONE		= 0x00,								/* 无网络. */
	SDK_NET_SAME		= 0x01,								/* 同网络. */
	SDK_NET_OTHER		= 0x02,								/* 其他网络. */
	SDK_NET_UNKNOWN		= 0x03,								/* 状态未知. */

	SDK_NET_DEF,											/* 无效状态. */
} sdk_net_state_t;

typedef enum _sdk_tst_cmd_t									/* 测试指令命令字. */
{
	SDK_TST_SFO			= 0x00,								/* 频偏同步. */
	SDK_TST_GET_SFO		= 0x01,								/* 读取频偏. */
	SDK_TST_REQ			= 0x02,								/* 性能测试数据发送请求. */
	SDK_TST_GAIN		= 0x03,								/* 读取RX GAIN/TX GAIN. */
	SDK_TST_GETNB		= 0x04,								/* 获取被测模块地址 */
	SDK_TST_CLKNB		= 0x05,								/* 清除被测模块地址 */

	SDK_TST_DEF,											/* 无效测试命令字. */
} sdk_tst_cmd_t;

typedef enum _sdk_upd_type_t
{
    SDK_UPDTP_NONE      = 0x00,                             /* 没有升级包 */
    SDK_UPDTP_LOW       = 0x01,                             /* 升级固件 */
    SDK_UPDTP_HIGH      = 0x02,                             /* 升级分层应用 */
    SDK_UPDTF_DEF                                           /* 无效升级包类型 */
} sdk_upd_type_t;

typedef struct _sdk_skt_t									/* 套接字. */
{
	u32					ip;									/* IP. */
	u16					port;								/* 端口. */
} sdk_skt_t;

typedef struct _sdk_skt_pair_t								/* 套接字对. */
{
	sdk_skt_t			src;								/* 源套接字. */
	sdk_skt_t			dst;								/* 目的套接字. */
} sdk_skt_pair_t;

typedef struct _sdk_link_state_t							/* 连接状态. */
{
	sdk_skt_pair_t		skt;								/* 套接字对. */
	sdk_link_sta_t		sta;								/* 状态. */
} sdk_link_state_t;

typedef struct _sdk_tmi_t									/* TMI参数. */
{
	u8					mm;									/* 调制方式. */
	u8					copy;								/* 重复次数, 取值1~12, 目前只支持1, 2, 4, 5, 7, 11. */
	u8					fec;								/* FEC编码率. */
} sdk_tmi_t;

typedef struct _sdk_frm_t									/* FRM帧长信息. */
{
	u8					pb_amt;								/* PB数量, 最大15个, 目前限制为4个. */
	u8					pb_type;							/* PB类项. 0-136pb, 1-520pb*/
} sdk_frm_t;

#endif
