#include "includes.h"

volatile uint32_t	TIM4_IRQCNT;				//TIM4中断计数器
volatile uint32_t	TIM2_IRQCNT;	
volatile uint32_t       MPU6050_Tim_1ms=0;
volatile uint32_t	time_1ms_cnt=0; 	//TIM3中断计数器
#define VECT_TAB_RAM
/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000		
*******************************************************************************/
void TIM4_INIT(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_DeInit(TIM4);
	//a period is 1000 times
	TIM_TimeBaseStructure.TIM_Period=1000;
	//prescaler is 1200,that is 72000000/72/1000=1000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//division number   is 72 
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	//clear the TIM2 overflow interrupt flag
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	//enable TIM4
	TIM_Cmd(TIM4,ENABLE);
}
/**************************实现函数********************************************
*函数原型:		
*功　　能:		延时  
*******************************************************************************/
void delay_ms(uint16_t nms)
{	
	TIM4->CNT = 0;
  	while(nms--)
	{
		while(TIM_GetITStatus(TIM4 , TIM_IT_Update) == RESET);
			TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);   //清除中断标志
	}
} 
void delay_us(uint16_t nus)
{	
	TIM4->CNT = 1000 - nus;
	while(TIM_GetITStatus(TIM4 , TIM_IT_Update) == RESET);
	TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //清除中断标志
} 

/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000	
time3  的最高时钟为32  M 
*******************************************************************************/
void TIM2_INIT(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_DeInit(TIM2);
	//a period is 1000 times
	TIM_TimeBaseStructure.TIM_Period=1000;
	//prescaler is 1200,that is 72000000/72/1000=1000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//division number   is 72 
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	//clear the TIM2 overflow interrupt flag
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//enable TIM2
	TIM_Cmd(TIM2,ENABLE);
}	
void DrYL_TIME2_NVIC_Configutation(void)
{
      NVIC_InitTypeDef NVIC_InitStructure;


      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择中断分组1
      NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //选择TIM2的中断通道      
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占式中断优先级设置为0
     // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应式中断优先级设置为0
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //使能中断
      
      NVIC_Init(&NVIC_InitStructure);

}

void TIM2_IRQ(void)
{
	TIM2_IRQCNT++;
        time_1ms_cnt++;
        MPU6050_Tim_1ms++;
        
}

