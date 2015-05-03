/*****************************************************************************************
圆点博士STM32蓝牙4.0最小系统板AnBT库文件和例程源代码。和圆点博士STM32无线下载助手软件兼容。
该例程适用LGPL协议。用户可以自由转载使用该代码，但不得移除该部分版权信息
更多信息，请访问官方网站www.etootle.com
******************************************************************************************/
//#include "stm32f10x_lib.h"
#include "anbt_i2c.h"
//#include "anbt_uart.h"
//
#define ANBT_HMC5883L_MAG_ADDR 					0x3C
#define ANBT_HMC5883L_MAG_WHOAMI_ADDR 	0x0A
//
#define ANBT_HMC5883L_CFG_A_ADDR 		0x00
#define ANBT_HMC5883L_CFG_A_VALUE 	0x70
//
#define ANBT_HMC5883L_CFG_B_ADDR 		0x01
#define ANBT_HMC5883L_CFG_B_VALUE 	0xA0
//
#define ANBT_HMC5883L_MODE_ADDR 	0x02
#define ANBT_HMC5883L_MODE_VALUE 	0x00
//
#define ANBT_HMC5883L_MAG_DATA_ADDR 	0x03
//
#define ANBT_HMC5883L_MAG_STATUS_ADDR 	0x09
//
u8 ANBT_HMC5883L_MAG_WHOAMI_FUN(void);
void ANBT_HMC5883L_MAG_Init_FUN(void);
u8 ANBT_HMC5883L_MAG_Get_Status_FUN(void);
void ANBT_HMC5883L_MAG_Read_Data_FUN(u8 *hmc5883l_mag_data_buffer);
void ANBT_HMC5883L_SEND_MAG_DATA_FUN(u8 *hmc5883l_mag_data_buffer);
void AnBT_HMC5883L_Delay(unsigned int nCount);

