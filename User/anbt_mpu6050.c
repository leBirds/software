/*************************************************************************************************************
圆点博士小四轴飞行器2014版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_mpu6050.h"
extern unsigned char anbt_ak8975_mag_data_buffer[6];

void ANBT_MPU6050_Init(void)	
{
	//AnBT_Uart1_Send_String("M-6,Init MPU6050 Device.",24);
	//
        ANBT_MPU6050_EXIT_SLEEP_FUN();    //圆点博士:退出休眠模式
	ANBT_MPU6050_PWM_CFG_FUN(); 	 //圆点博士:设置内部时钟
	ANBT_MPU6050_GYRO_CFG_FUN();      //圆点博士:设置陀螺仪量程
	ANBT_MPU6050_ACCEL_CFG_FUN();     //圆点博士:设置加速度量程
	ANBT_MPU6050_USER_CTRL_FUN();
	ANBT_MPU6050_I2CBYPASS_CFG_FUN(); //圆点博士:设置电磁读写模式
	ANBT_MPU6050_INT_CFG_FUN();
}

void ANBT_SEND_CHIP_IDs_FUN(u8 mpu6050_gyro_id,u8 hmc5883l_mag_id,u8 ak8975_mag_id)	
{
	unsigned char chip_id[3];
	unsigned char data_type,checksum,i;
	//
	AnBT_Uart1_Send_String("M-7,Send chip IDs.",18);
	//
	chip_id[0]=mpu6050_gyro_id;
	chip_id[1]=hmc5883l_mag_id;	
	chip_id[2]=ak8975_mag_id;	
	//
	data_type=0xA0;
	checksum=data_type;
	for(i=0;i<3;i++) checksum+=chip_id[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//
	AnBT_Uart1_Send_Char(':');
	AnBT_Uart1_Send_Num(data_type);
	for(i=0;i<3;i++) AnBT_Uart1_Send_Num(chip_id[i]);	
	AnBT_Uart1_Send_Num(checksum);
	AnBT_Uart1_Send_Char('/');
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
}

u8 ANBT_MPU6050_GYRO_WHOAMI_FUN(void)
{
	u8 anbt_mpu6050_gyro_id;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_WHOAMI_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR+1);      //圆点博士:发送陀螺仪读地址
	anbt_mpu6050_gyro_id=ANBT_I2C_ReceiveByte();	//圆点博士:读出陀螺仪ID
	ANBT_I2C_STOP();
	//
	return anbt_mpu6050_gyro_id;
}
//
u8 ANBT_MPU6050_READ_REG_FUN(u8 anbt_mpu6050_dev_addr,u8 anbt_mpu6050_reg_addr)   
{
	u8 anbt_mpu6050_reg;
	
	ANBT_I2C_START();
	ANBT_I2C_SendByte(anbt_mpu6050_dev_addr);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(anbt_mpu6050_reg_addr);    //圆点博士:发送陀螺仪寄存器地址
	ANBT_I2C_START();
	ANBT_I2C_SendByte(anbt_mpu6050_dev_addr+1);      //圆点博士:发送陀螺仪读地址
	anbt_mpu6050_reg=ANBT_I2C_ReceiveByte();	//圆点博士:读出陀螺仪寄存器值
	ANBT_I2C_STOP();
	//
	return anbt_mpu6050_reg;
}

void ANBT_MPU6050_PWM_CFG_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_PWR_MGMT_1_VALUE); //圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}
//
void ANBT_MPU6050_GYRO_CFG_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);		//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_CFG_ADDR);   	//圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}
//
void ANBT_MPU6050_ACCEL_CFG_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_ACCEL_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_ACCEL_CFG_VALUE); //圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}

void ANBT_MPU6050_EXIT_SLEEP_FUN(void)  
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_EXIT_SLEEP_VALUE); //圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}

void ANBT_MPU6050_USER_CTRL_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_USER_CTRL_ADDR);  //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_USER_CTRL_VALUE); //圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}

void ANBT_MPU6050_I2CBYPASS_CFG_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);		//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_I2CBYPASS_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_I2CBYPASS_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}

void ANBT_MPU6050_INT_CFG_FUN(void)   
{
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_MPU6050_INT_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	ANBT_I2C_SendByte(ANBT_MPU6050_INT_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	ANBT_I2C_STOP();
}

void ANBT_MPU6050_GYRO_READ_FUN(u8 *mpu6050_gyro_data_buffer)   
{
	if(!ANBT_MPU6050_INT_STATE) 
	{
		ANBT_I2C_START();
		ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR);	//圆点博士:发送陀螺仪写地址
		ANBT_I2C_SendByte(ANBT_MPU6050_ACCEL_DATA_ADDR);   //圆点博士:发送陀螺仪寄存器地址
		ANBT_I2C_START();
		ANBT_I2C_SendByte(ANBT_MPU6050_GYRO_ADDR+1);      //圆点博士:发送陀螺仪读地址
		ANBT_I2C_Receive14Bytes(mpu6050_gyro_data_buffer); //圆点博士:读出陀螺仪寄存器值
		ANBT_I2C_STOP();
	}
}

void ANBT_MPU6050_SEND_DATA_FUN(u8 *mpu6050_gyro_data_buffer)	
{
	unsigned char data_type,checksum,i;
	//
	data_type=0xA1;
	checksum=data_type;
	for(i=0;i<14;i++) checksum+=mpu6050_gyro_data_buffer[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//
	AnBT_Uart1_Send_Char(':');
	AnBT_Uart1_Send_Num(data_type);
	for(i=0;i<14;i++) AnBT_Uart1_Send_Num(mpu6050_gyro_data_buffer[i]);	
	AnBT_Uart1_Send_Num(checksum);
	AnBT_Uart1_Send_Char('/');
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
}
