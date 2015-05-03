/*************************************************************************************************************
圆点博士小四轴飞行器2014版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_hmc5883l.h"

u8 ANBT_HMC5883L_MAG_WHOAMI_FUN(void)
{
	u8 anbt_hmc5883l_mag_id;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_WHOAMI_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //圆点博士:发送陀螺仪读地址
	anbt_hmc5883l_mag_id=ANBT_I2C_ReceiveByte();				//圆点博士:读出陀螺仪ID
	ANBT_I2C_STOP();
	//
	return anbt_hmc5883l_mag_id;
}
//
void ANBT_HMC5883L_MAG_Init_FUN(void)
{
        ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_A_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_A_VALUE);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_B_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_B_VALUE);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_MODE_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_MODE_VALUE);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
}

u8 ANBT_HMC5883L_MAG_Get_Status_FUN(void)
{
	u8 anbt_hmc5883l_mag_ready;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_STATUS_ADDR);  //圆点博士:发送陀螺仪ID地址
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //圆点博士:发送陀螺仪读地址
	anbt_hmc5883l_mag_ready=ANBT_I2C_ReceiveByte();	//圆点博士:读出陀螺仪ID
	ANBT_I2C_STOP();
	//
	return anbt_hmc5883l_mag_ready;
}

void ANBT_HMC5883L_MAG_Read_Data_FUN(u8 *hmc5883l_mag_data_buffer)
{
	u8 anbt_hmc5883l_mag_data_ready;
	//	
	AnBT_HMC5883L_Delay(10000);
	anbt_hmc5883l_mag_data_ready=ANBT_HMC5883L_MAG_Get_Status_FUN();
	if(anbt_hmc5883l_mag_data_ready&0x01)
	{
		ANBT_I2C_START();
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//圆点博士:发送陀螺仪写地址
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_DATA_ADDR);  										//圆点博士:发送陀螺仪ID地址
		ANBT_I2C_START();
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //圆点博士:发送陀螺仪读地址
		ANBT_I2C_Receive6Bytes(hmc5883l_mag_data_buffer);
		ANBT_I2C_STOP();
	}
}

void ANBT_HMC5883L_SEND_MAG_DATA_FUN(u8 *hmc5883l_mag_data_buffer)	
{
	unsigned char data_type,checksum,i;
	//
	data_type=0xA2;
	checksum=data_type;
	for(i=0;i<6;i++) checksum+=hmc5883l_mag_data_buffer[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//
	//AnBT_Uart1_Send_Char(':');
	//AnBT_Uart1_Send_Num(data_type);
	//for(i=0;i<6;i++) AnBT_Uart1_Send_Num(hmc5883l_mag_data_buffer[i]);	
	//AnBT_Uart1_Send_Num(checksum);
	//AnBT_Uart1_Send_Char('/');
	//AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
}

void AnBT_HMC5883L_Delay(unsigned int nCount) 		  //圆点博士:延时函数
{
	for(; nCount != 0; nCount--);
}
