#ifndef __ANBT_UART_H__
#define __ANBT_UART_H__
/*****************************************************************************************
Բ�㲩ʿSTM32����4.0��Сϵͳ��AnBT���ļ�������Դ���롣��Բ�㲩ʿSTM32������������������ݡ�
����������LGPLЭ�顣�û���������ת��ʹ�øô��룬�������Ƴ��ò��ְ�Ȩ��Ϣ
������Ϣ������ʹٷ���վwww.etootle.com
******************************************************************************************/
//#include "stm32f10x_lib.h"
#include "includes.h"

#define AnBT_COM_Buf_Length	64
#define AnBT_USART3_TX	GPIO_Pin_10
#define AnBT_USART3_RX	GPIO_Pin_11


#define DrYL_USART2_TX  GPIO_Pin_2
#define DrYL_USART2_RX  GPIO_Pin_3
//
extern unsigned char com_receive_str_index;		//Բ�㲩ʿ:���ջ����ַ����
extern unsigned char com_receive_str_buf[AnBT_COM_Buf_Length]; //Բ�㲩ʿ:���ջ���
//
extern unsigned char pid_data_buffer[16];
extern unsigned char motor_unlock_sign;
extern unsigned char gas_data_buffer;
extern unsigned char pitch_data_buffer,roll_data_buffer,yaw_data_buffer;
//
extern void DrYL_Uart3_RCC_Configuration(void);
extern void AnBT_UART3_Configuration(void);
extern void AnBT_UART3_NVIC_Configuration(void);

extern void AnBT_UART3_GPIO_Configuration(void);
extern void AnBT_Uart3_Send_Char(unsigned char ascii_code);
extern void AnBT_Uart3_Send_String(unsigned char* str_buf , unsigned char str_len);
extern void AnBT_Uart3_Send_Num(unsigned char number);
extern void AnBT_Uart3_Send_Nums(unsigned char* nums_buf , unsigned char nums_len);
extern void DrYL_Uart3_Send_Uint(unsigned int union_code);
//

extern void DrYL_Uart2_RCC_Configuration(void);
extern void AnBT_UART2_GPIO_Configuration(void);
extern void AnBT_UART2Configuration(void);
extern void AnBT_UART2_NVIC_Configuration(void);
extern void AnBT_Uart2_Send_Char(unsigned char ascii_code);
extern void AnBT_UART2_Interrupt(void);
#endif
