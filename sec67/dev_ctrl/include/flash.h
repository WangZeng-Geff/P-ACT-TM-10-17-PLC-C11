#ifndef __FLASH_H__
#define __FLASH_H__
#include "types.h"

/**************************************************
*                 flash map
*
*                0 ----------
*       bootloader   |     |			//56K
*           0xE000 ----------
*	    bootinfo     |     |			//8K
*          0x10000 ----------
*       MAC addr     |     |			//8K
*		   0x12000 ----------
*       Run image    |     |			//384K
*          0x72000 ----------
*       Back image   |     |			//192K
*          0xA2000 ----------
*       Update image |     |			//192K
*          0xD2000 ----------
*       data         |     |			//8K
*          0xD4000 ----------
*       Sec high     |     |			//96K
*          0xEC000 ----------
*       Sec high back|     |			//48K
*          0xF8000 ----------
*       Data info    |     |			//32K
*              end ----------
*
***************************************************************/
#define UPDATE_ADDR			0xA2000
#define UPDATE_LEN			0x30000
#define USER_ADDR           0xF8000
#define USER_LEN            0x7FFF

#define DEVICE_INFO_ADDR	USER_ADDR
#define DEVICE_INFO_LEN		0x100
#define DEVICE_INFO_MAGIC	0x83676800

#define DEVICE_CFG_ADDR		(DEVICE_INFO_ADDR + DEVICE_INFO_LEN)
#define DEVICE_CFG_LEN		0x100
#define DEVICE_CFG_MAGIC	0x55AA2B2B

#define AIR_DATA_ADDR 		(DEVICE_CFG_ADDR + DEVICE_CFG_LEN)
#define AIR_DATA_LEN		0x200
#define AIR_DATA_MAGIC		0x55AA2B2B

#define EEP_MOTOR_ADDR 		(AIR_DATA_ADDR + AIR_DATA_LEN)
#define EEP_MOTOR_LEN		0x200
#define EEP_MOTOR_MAGIC		0x55AA2B2B

#define offset_of(obj_type,mb)  ((unsigned int)&(((obj_type*)0)->mb))


/***********************************************************/
/*
   由于flash不能调用ram中的其他函数，
   所以外部调用按照返回值判定flash错误原因
   由于写操作flash的时候不能产生有其他访问flash的操作
*/
/***********************************************************/
#define FLASH_OK        (int)(0)
#define FERR_ADDR       (int)(-1)
#define FERR_IAP        (int)(-2)
#define FERR_INCON      (int)(-3)
#define FERR_PARA       (int)(-4)

/*****************************************************************************************************/
/*
	Function	: flash_image_bin_erase
	Description	: 
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern int32_t flash_image_bin_erase(uint32_t addr, uint32_t len);

/*****************************************************************************************************/
/*
	Function	: flash_user_info_erase
	Description	: 
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern int32_t flash_user_info_erase(uint32_t addr, uint32_t len);

/*****************************************************************************************************/
/*
	Function	: flash_image_bin_write
	Description	: 
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern int32_t flash_image_bin_write(uint32_t dst, uint8_t *psrc, uint32_t len);

/*****************************************************************************************************/
/*
	Function	: flash_user_info_write
	Description	: 
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern int32_t flash_user_info_write(uint32_t dst, uint8_t *psrc, uint32_t len);
/*****************************************************************************************************/
/*
	Function	:
	Description	:
	Input		:
	Output		:
	Return		:
	Notes		:
*/
/*****************************************************************************************************/
extern int32_t flash_user_info_read(unsigned int addr, unsigned char buf[], unsigned int len);

/*****************************************************************************************************/

#endif /* end of flash.h */
