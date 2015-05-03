/*************************************************************************************************************
圆点博士小四轴飞行器2014版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "anbt_uart.h"
#include "anbt_uart_def.h"

unsigned char com_receive_str_index;		//圆点博士:接收缓冲地址索引
unsigned char com_receive_str_buf[AnBT_COM_Buf_Length]; //圆点博士:接收缓冲
//
unsigned char pid_data_buffer[16];
unsigned char motor_unlock_sign;
unsigned char gas_data_buffer;
unsigned char pitch_data_buffer,roll_data_buffer,yaw_data_buffer;


void  DrYL_Uart3_RCC_Configuration(void)
{
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //串口时钟配置
}
void AnBT_UART3_GPIO_Configuration(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = AnBT_USART3_TX;					//圆点博士:设置PA9管脚为串口TX
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//圆点博士:设置串口TX最大允许输出速度
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   		//圆点博士:设置串口TX为输出
        GPIO_Init(GPIOB, &GPIO_InitStructure);
              //
        GPIO_InitStructure.GPIO_Pin = AnBT_USART3_RX;		//圆点博士:设置PA9管脚为串口RX
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //圆点博士:设置串口RX为输入
        GPIO_Init(GPIOB, &GPIO_InitStructure);  
              //
        //AnBT_Uart1_Send_String("M-1,Init COM Device.",20);
}

void AnBT_UART3_Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;
      
      USART_InitStructure.USART_BaudRate = 9600;			//圆点博士:设置串口波特率为9600
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //圆点博士:设置串口数据长度为8位
      USART_InitStructure.USART_StopBits = USART_StopBits_1;        //圆点博士:设置串口停止位长度为1位
      USART_InitStructure.USART_Parity = USART_Parity_No ;		    //圆点博士:设置串口奇偶校验为无
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //圆点博士:设置串口数据流控制为无
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//圆点博士:设置串口为发送和接收模式
      USART_Init(USART3, &USART_InitStructure);			//圆点博士:设置串口参数
      USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);	        //圆点博士:允许接收中断
      USART_Cmd(USART3, ENABLE);  				       //圆点博士:使能串口
}

void AnBT_UART3_NVIC_Configuration(void)				//圆点博士:设置串口中断优先级
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
        去掉以后不能接收，只能发送
        *****************************************************************************************/
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =USART3_IRQn;// USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void AnBT_Uart3_Send_Char(unsigned char ascii_code) 		//圆点博士:发送一个字符
{
	USART_SendData(USART3,ascii_code);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);							//圆点博士:等待直到发送完成
}

void AnBT_Uart3_Send_String(unsigned char* str_buf , unsigned char str_len)		//圆点博士:发送一个指定长度的字符串
{
	/*unsigned char i;
	if(str_len>AnBT_COM_Buf_Length) str_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
	AnBT_Uart1_Send_Char(':');  															//圆点博士:发送字符:
        for(i=0;i<str_len;i++) AnBT_Uart1_Send_Char(str_buf[i]); 	//圆点博士:发送字符:
	AnBT_Uart1_Send_Char('/');																//圆点博士:发送字符/
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
*/
}
void DrYL_Uart3_Send_Uint(unsigned int union_code)
{
      AnBT_Uart3_Send_Char((union_code&0xff00)>>8);
      AnBT_Uart3_Send_Char(union_code&0x00ff);
}

void AnBT_Uart3_Send_Num(unsigned char number) 	//圆点博士:发送一个字符
{
	/*unsigned char num_low,num_high;
	num_low=number&0x0f;													//圆点博士:取数据低位
	num_high=(number&0xf0)>>4;										//圆点博士:取数据高位
	if(num_high<10)USART_SendData(USART1,num_high+48);
	else USART_SendData(USART1,num_high+55);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
	if(num_low<10)USART_SendData(USART1,num_low+48);
	else USART_SendData(USART3,num_low+55);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
    */
}

void AnBT_Uart3_Send_Nums(unsigned char* nums_buf , unsigned char nums_len)		//圆点博士:发送一个指定长度的字符串
{
	/*unsigned char i;
	if(nums_len>AnBT_COM_Buf_Length) nums_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																										//圆点博士:发送回车字符
	AnBT_Uart1_Send_Char(':');  																								//圆点博士:发送字符:
        for(i=0;i<nums_len;i++) AnBT_Uart1_Send_Num(nums_buf[nums_len-i-1]); 				//圆点博士:发送数字
	AnBT_Uart3_Send_Char('/');																									//圆点博士:发送字符/
*/
}




void  DrYL_Uart2_RCC_Configuration(void)
{
      // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //使能UART3所在GPIOB的时钟
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //串口时钟配置
}
void AnBT_UART2_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = DrYL_USART2_TX;			//圆点博士:设置PA2管脚为串口TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//圆点博士:设置串口TX最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   		//圆点博士:设置串口TX为输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//
  GPIO_InitStructure.GPIO_Pin = DrYL_USART2_RX;		//圆点博士:设置PA3管脚为串口RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //圆点博士:设置串口RX为输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//
  //AnBT_Uart1_Send_String("M-1,Init COM Device.",20);
}
void AnBT_UART2Configuration(void)
{
      USART_InitTypeDef USART_InitStructure;
      //USART_ClockInitTypeDef USART_ClockInitStruct;
            //
      USART_InitStructure.USART_BaudRate = 9600;			//圆点博士:设置串口波特率为9600
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //圆点博士:设置串口数据长度为8位
      USART_InitStructure.USART_StopBits = USART_StopBits_1;        //圆点博士:设置串口停止位长度为1位
      USART_InitStructure.USART_Parity = USART_Parity_No ;		//圆点博士:设置串口奇偶校验为无
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //圆点博士:设置串口数据流控制为无
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      //圆点博士:设置串口为发送和接收模式
      USART_Init(USART2, &USART_InitStructure);	
      
      //USART_ClockInit(USART2, &USART_ClockInitStruct);//圆点博士:设置串口参数
      USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	        //圆点博士:允许接收中断
      
      USART_Cmd(USART2,ENABLE);                          //圆点博士:使能串口			       

}
void AnBT_UART2_NVIC_Configuration(void)				//圆点博士:设置串口中断优先级
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
void AnBT_Uart2_Send_Char(unsigned char ascii_code) 		//圆点博士:发送一个字符
{
	USART_SendData(USART2,ascii_code);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);							//圆点博士:等待直到发送完成
}
u8 com_receive_data;
void AnBT_UART2_Interrupt(void)
{
    if(USART_GetFlagStatus(USART2,USART_IT_RXNE) == SET)	//圆点博士:发生的是串口接收中断
    {
        
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
    { 
        com_receive_data=USART_ReceiveData(USART2);
        AnBT_Uart2_Send_Char('a');
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
        
}