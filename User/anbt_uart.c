/*************************************************************************************************************
Բ�㲩ʿС���������2014������Դ��������:
��Դ��������ο�,Բ�㲩ʿ����Դ�����ṩ�κ���ʽ�ĵ���,Ҳ������ʹ�ø�Դ��������ֵ���ʧ����.
�û�������ѧϰ��Ŀ���޸ĺ�ʹ�ø�Դ����.
���û����޸ĸ�Դ����ʱ,�����Ƴ��ò��ְ�Ȩ��Ϣ�����뱣��ԭ������.

������Ϣ������ʹٷ���վwww.etootle.com, �ٷ�����:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_uart.h"
#include "anbt_uart_def.h"

unsigned char com_receive_str_index;		//Բ�㲩ʿ:���ջ����ַ����
unsigned char com_receive_str_buf[AnBT_COM_Buf_Length]; //Բ�㲩ʿ:���ջ���
//
unsigned char pid_data_buffer[16];
unsigned char motor_unlock_sign;
unsigned char gas_data_buffer;
unsigned char pitch_data_buffer,roll_data_buffer,yaw_data_buffer;


void  DrYL_Uart3_RCC_Configuration(void)
{
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3����GPIOB��ʱ��
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //����ʱ������
}
void AnBT_UART3_GPIO_Configuration(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = AnBT_USART3_TX;					//Բ�㲩ʿ:����PA9�ܽ�Ϊ����TX
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//Բ�㲩ʿ:���ô���TX�����������ٶ�
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   		//Բ�㲩ʿ:���ô���TXΪ���
        GPIO_Init(GPIOB, &GPIO_InitStructure);
              //
        GPIO_InitStructure.GPIO_Pin = AnBT_USART3_RX;		//Բ�㲩ʿ:����PA9�ܽ�Ϊ����RX
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Բ�㲩ʿ:���ô���RXΪ����
        GPIO_Init(GPIOB, &GPIO_InitStructure);  
              //
        //AnBT_Uart1_Send_String("M-1,Init COM Device.",20);
}

void AnBT_UART3_Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;
      
      USART_InitStructure.USART_BaudRate = 9600;			//Բ�㲩ʿ:���ô��ڲ�����Ϊ9600
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //Բ�㲩ʿ:���ô������ݳ���Ϊ8λ
      USART_InitStructure.USART_StopBits = USART_StopBits_1;        //Բ�㲩ʿ:���ô���ֹͣλ����Ϊ1λ
      USART_InitStructure.USART_Parity = USART_Parity_No ;		    //Բ�㲩ʿ:���ô�����żУ��Ϊ��
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //Բ�㲩ʿ:���ô�������������Ϊ��
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//Բ�㲩ʿ:���ô���Ϊ���ͺͽ���ģʽ
      USART_Init(USART3, &USART_InitStructure);			//Բ�㲩ʿ:���ô��ڲ���
      USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);	        //Բ�㲩ʿ:��������ж�
      USART_Cmd(USART3, ENABLE);  				       //Բ�㲩ʿ:ʹ�ܴ���
}

void AnBT_UART3_NVIC_Configuration(void)				//Բ�㲩ʿ:���ô����ж����ȼ�
{
        NVIC_InitTypeDef NVIC_InitStructure;
        /*****************************************************************************************/
        #ifdef  VECT_TAB_RAM
        /* Set the Vector Table base location at 0x20000000 */
        NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x00);
        #else  /* VECT_TAB_FLASH  */
            /* Set the Vector Table base location at 0x08000000 */
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
        #endif
        /***************************************************************************************
        ȥ���Ժ��ܽ��գ�ֻ�ܷ���
        *****************************************************************************************/
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =USART3_IRQn;// USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void AnBT_Uart3_Send_Char(unsigned char ascii_code) 		//Բ�㲩ʿ:����һ���ַ�
{
	USART_SendData(USART3,ascii_code);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);							//Բ�㲩ʿ:�ȴ�ֱ���������
}

void AnBT_Uart3_Send_String(unsigned char* str_buf , unsigned char str_len)		//Բ�㲩ʿ:����һ��ָ�����ȵ��ַ���
{
	/*unsigned char i;
	if(str_len>AnBT_COM_Buf_Length) str_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																	//Բ�㲩ʿ:���ͻس��ַ�
	AnBT_Uart1_Send_Char(':');  															//Բ�㲩ʿ:�����ַ�:
        for(i=0;i<str_len;i++) AnBT_Uart1_Send_Char(str_buf[i]); 	//Բ�㲩ʿ:�����ַ�:
	AnBT_Uart1_Send_Char('/');																//Բ�㲩ʿ:�����ַ�/
	AnBT_Uart1_Send_Char(13);																	//Բ�㲩ʿ:���ͻس��ַ�
*/
}
void DrYL_Uart3_Send_Uint(unsigned int union_code)
{
      AnBT_Uart3_Send_Char((union_code&0xff00)>>8);
      AnBT_Uart3_Send_Char(union_code&0x00ff);
}

void AnBT_Uart3_Send_Num(unsigned char number) 	//Բ�㲩ʿ:����һ���ַ�
{
	/*unsigned char num_low,num_high;
	num_low=number&0x0f;													//Բ�㲩ʿ:ȡ���ݵ�λ
	num_high=(number&0xf0)>>4;										//Բ�㲩ʿ:ȡ���ݸ�λ
	if(num_high<10)USART_SendData(USART1,num_high+48);
	else USART_SendData(USART1,num_high+55);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//Բ�㲩ʿ:�ȴ�ֱ���������
	if(num_low<10)USART_SendData(USART1,num_low+48);
	else USART_SendData(USART3,num_low+55);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}								//Բ�㲩ʿ:�ȴ�ֱ���������
    */
}

void AnBT_Uart3_Send_Nums(unsigned char* nums_buf , unsigned char nums_len)		//Բ�㲩ʿ:����һ��ָ�����ȵ��ַ���
{
	/*unsigned char i;
	if(nums_len>AnBT_COM_Buf_Length) nums_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																										//Բ�㲩ʿ:���ͻس��ַ�
	AnBT_Uart1_Send_Char(':');  																								//Բ�㲩ʿ:�����ַ�:
        for(i=0;i<nums_len;i++) AnBT_Uart1_Send_Num(nums_buf[nums_len-i-1]); 				//Բ�㲩ʿ:��������
	AnBT_Uart3_Send_Char('/');																									//Բ�㲩ʿ:�����ַ�/
*/
}




void  DrYL_Uart2_RCC_Configuration(void)
{
      // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //ʹ��UART3����GPIOB��ʱ��
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //����ʱ������
}
void AnBT_UART2_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = DrYL_USART2_TX;			//Բ�㲩ʿ:����PA2�ܽ�Ϊ����TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//Բ�㲩ʿ:���ô���TX�����������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   		//Բ�㲩ʿ:���ô���TXΪ���
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = DrYL_USART2_RX;		//Բ�㲩ʿ:����PA3�ܽ�Ϊ����RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //Բ�㲩ʿ:���ô���RXΪ����
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//
  //AnBT_Uart1_Send_String("M-1,Init COM Device.",20);
}
void AnBT_UART2Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;
      //USART_ClockInitTypeDef USART_ClockInitStruct;
            //
      USART_InitStructure.USART_BaudRate = 9600;			//Բ�㲩ʿ:���ô��ڲ�����Ϊ9600
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //Բ�㲩ʿ:���ô������ݳ���Ϊ8λ
      USART_InitStructure.USART_StopBits = USART_StopBits_1;        //Բ�㲩ʿ:���ô���ֹͣλ����Ϊ1λ
      USART_InitStructure.USART_Parity = USART_Parity_No ;		//Բ�㲩ʿ:���ô�����żУ��Ϊ��
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //Բ�㲩ʿ:���ô�������������Ϊ��
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      //Բ�㲩ʿ:���ô���Ϊ���ͺͽ���ģʽ
      USART_Init(USART2, &USART_InitStructure);	
      
      //USART_ClockInit(USART2, &USART_ClockInitStruct);//Բ�㲩ʿ:���ô��ڲ���
      USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	        //Բ�㲩ʿ:��������ж�
      
      USART_Cmd(USART2,ENABLE);                          //Բ�㲩ʿ:ʹ�ܴ���			       

}
void AnBT_UART2_NVIC_Configuration(void)				//Բ�㲩ʿ:���ô����ж����ȼ�
{
	NVIC_InitTypeDef NVIC_InitStructure;
        #ifdef  VECT_TAB_RAM
        /* Set the Vector Table base location at 0x20000000 */
        NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
        #else  /* VECT_TAB_FLASH  */
            /* Set the Vector Table base location at 0x08000000 */
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
        #endif
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel =USART2_IRQn;// USART2_IRQChannel;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void AnBT_Uart2_Send_Char(unsigned char ascii_code) 		//Բ�㲩ʿ:����һ���ַ�
{
	USART_SendData(USART2,ascii_code);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);							//Բ�㲩ʿ:�ȴ�ֱ���������
}
u8 com_receive_data;
void AnBT_UART2_Interrupt(void)
{
    if(USART_GetFlagStatus(USART2,USART_IT_RXNE) == SET)	//Բ�㲩ʿ:�������Ǵ��ڽ����ж�
    {
        
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
    { 
        com_receive_data=USART_ReceiveData(USART2);
        AnBT_Uart2_Send_Char('a');
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
        
}