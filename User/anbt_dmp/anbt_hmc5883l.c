/*************************************************************************************************************
Բ�㲩ʿС���������2014������Դ��������:
��Դ��������ο�,Բ�㲩ʿ����Դ�����ṩ�κ���ʽ�ĵ���,Ҳ������ʹ�ø�Դ��������ֵ���ʧ����.
�û�������ѧϰ��Ŀ���޸ĺ�ʹ�ø�Դ����.
���û����޸ĸ�Դ����ʱ,�����Ƴ��ò��ְ�Ȩ��Ϣ�����뱣��ԭ������.

������Ϣ������ʹٷ���վwww.etootle.com, �ٷ�����:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_hmc5883l.h"

u8 ANBT_HMC5883L_MAG_WHOAMI_FUN(void)
{
	u8 anbt_hmc5883l_mag_id;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_WHOAMI_ADDR);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //Բ�㲩ʿ:���������Ƕ���ַ
	anbt_hmc5883l_mag_id=ANBT_I2C_ReceiveByte();				//Բ�㲩ʿ:����������ID
	ANBT_I2C_STOP();
	//
	return anbt_hmc5883l_mag_id;
}
//
void ANBT_HMC5883L_MAG_Init_FUN(void)
{
        ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_A_ADDR);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_A_VALUE);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_B_ADDR);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_CFG_B_VALUE);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_MODE_ADDR);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_MODE_VALUE);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_STOP();
	ANBT_I2C_DELAY;
	ANBT_I2C_DELAY;
}

u8 ANBT_HMC5883L_MAG_Get_Status_FUN(void)
{
	u8 anbt_hmc5883l_mag_ready;
	//
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_STATUS_ADDR);  //Բ�㲩ʿ:����������ID��ַ
	ANBT_I2C_START();
	ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //Բ�㲩ʿ:���������Ƕ���ַ
	anbt_hmc5883l_mag_ready=ANBT_I2C_ReceiveByte();	//Բ�㲩ʿ:����������ID
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
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR);			//Բ�㲩ʿ:����������д��ַ
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_DATA_ADDR);  										//Բ�㲩ʿ:����������ID��ַ
		ANBT_I2C_START();
		ANBT_I2C_SendByte(ANBT_HMC5883L_MAG_ADDR+1);      //Բ�㲩ʿ:���������Ƕ���ַ
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
	//AnBT_Uart1_Send_Char(13);																	//Բ�㲩ʿ:���ͻس��ַ�
}

void AnBT_HMC5883L_Delay(unsigned int nCount) 		  //Բ�㲩ʿ:��ʱ����
{
	for(; nCount != 0; nCount--);
}
