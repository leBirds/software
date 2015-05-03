#ifndef __ANBT_MPU6050_H__
#define __ANBT_MPU6050_H__
/*****************************************************************************************
圆点博士STM32蓝牙4.0最小系统板AnBT库文件和例程源代码。和圆点博士STM32无线下载助手软件兼容。
该例程适用LGPL协议。用户可以自由转载使用该代码，但不得移除该部分版权信息
更多信息，请访问官方网站www.etootle.com
******************************************************************************************/
//#include "stm32f10x_lib.h"
#include "includes.h"
#include "anbt_i2c.h"
#include "anbt_uart.h"
//
#define ANBT_MPU6050_GYRO_ADDR 				0xD0
#define ANBT_MPU6050_GYRO_WHOAMI_ADDR                   0x75
//
#define ANBT_MPU6050_USER_CTRL_ADDR		        0x6A
#define ANBT_MPU6050_USER_CTRL_VALUE	                0x00
//
#define ANBT_MPU6050_PWR_MGMT_1_ADDR		        0x6B
#define ANBT_MPU6050_PWR_MGMT_1_VALUE 	                0x01
#define ANBT_MPU6050_EXIT_SLEEP_VALUE 	                0x00
//
#define ANBT_MPU6050_GYRO_CFG_ADDR 			0x1B
#define ANBT_MPU6050_GYRO_CFG_VALUE 		        0x18
//
#define ANBT_MPU6050_ACCEL_CFG_ADDR 		        0x1C
#define ANBT_MPU6050_ACCEL_CFG_VALUE 		        0x01
//
#define ANBT_MPU6050_I2CBYPASS_CFG_ADDR 		0x37
#define ANBT_MPU6050_I2CBYPASS_CFG_VALUE 		0xB2		//bit7=1, bit6=0, bit5=1, bit4=1, bit3=0, bit2=0, bit1=1, bit0=0
//
#define ANBT_MPU6050_INT_CFG_ADDR 		        0x38
#define ANBT_MPU6050_INT_CFG_VALUE 		        0x01
//
#define ANBT_MPU6050_ACCEL_DATA_ADDR 		        0x3B
//
extern u8 ANBT_MPU6050_GYRO_WHOAMI_FUN(void);
extern u8 ANBT_MPU6050_READ_REG_FUN(u8 anbt_mpu6050_dev_addr,u8 anbt_mpu6050_reg_addr);
extern void ANBT_MPU6050_PWM_CFG_FUN(void);
extern void ANBT_MPU6050_GYRO_CFG_FUN(void);
extern void ANBT_MPU6050_ACCEL_CFG_FUN(void);
extern void ANBT_MPU6050_EXIT_SLEEP_FUN(void);
extern void ANBT_MPU6050_I2CBYPASS_CFG_FUN(void);
extern void ANBT_MPU6050_USER_CTRL_FUN(void); 
extern void ANBT_MPU6050_INT_CFG_FUN(void);
extern void ANBT_MPU6050_Init(void);
extern void ANBT_MPU6050_SEND_DATA_FUN(u8 *MPU6050_gyro_data_buffer);
extern void ANBT_MPU6050_GYRO_READ_FUN(u8 *mpu6050_gyro_data_buffer);
extern void ANBT_SEND_CHIP_IDs_FUN(u8 mpu6050_gyro_id,u8 hmc5883l_mag_id,u8 ak8975_mag_id);


   
#endif