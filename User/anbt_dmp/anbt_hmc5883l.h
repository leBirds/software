/*****************************************************************************************
Բ�㲩ʿSTM32����4.0��Сϵͳ��AnBT���ļ�������Դ���롣��Բ�㲩ʿSTM32������������������ݡ�
����������LGPLЭ�顣�û���������ת��ʹ�øô��룬�������Ƴ��ò��ְ�Ȩ��Ϣ
������Ϣ������ʹٷ���վwww.etootle.com
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

