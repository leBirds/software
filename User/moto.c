#include "includes.h"

void Tim3_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;
	/* -----------------------------------------------------------------------
        TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
        The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
        clock at 24 MHz the Prescaler is computed as following:
            - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
        SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
        and Connectivity line devices and to 24 MHz for Low-Density Value line and
        Medium-Density Value line devices
            
            TIM3 counter clock is set 24M
        The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
            = 24 MHz / 1000 = 24 KHz
        TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
        TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
        TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
        TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
	----------------------------------------------------------------------- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 19999;		//计数上线	250Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//PrescalerValue;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1999;//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1999;
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse =1999;
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1999;
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

void STM32_PWM_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //打开外设A的时钟
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

	Tim3_init();	
}
/***************************************************************
*参数从左到右依次为up down left 和 right
*按照MPU6050的坐标给定，依次为+X，-X，+Y，-Y；
******************************************************************/

void MotoPWMControl(unsigned int moto_x_p,unsigned int moto_x_n,unsigned int moto_y_p,unsigned int moto_y_n)
{		
	if(moto_x_p   > Moto_PwmMax)	moto_x_p    = Moto_PwmMax;
	if(moto_x_n   > Moto_PwmMax)	moto_x_n    = Moto_PwmMax;
	if(moto_y_p   > Moto_PwmMax)	moto_y_p    = Moto_PwmMax;
	if(moto_y_n   > Moto_PwmMax)    moto_y_n    = Moto_PwmMax;
	if(moto_x_p   <=Moto_PwmMin)    moto_x_p    = Moto_PwmMin;
	if(moto_x_n   <=Moto_PwmMin)    moto_x_n    = Moto_PwmMin;
	if(moto_y_p   <=Moto_PwmMin)    moto_y_p    = Moto_PwmMin;
	if(moto_y_n   <=Moto_PwmMin)    moto_y_n    = Moto_PwmMin;
	
	TIM3->CCR1 = moto_x_p;
	TIM3->CCR2 = moto_x_n;
	TIM3->CCR4 = moto_y_n;
	TIM3->CCR3 = moto_y_p;
}

u16 Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative;
void DrYL_Motor_Driver_init(void)
{
      
        Moto_X_Positive   = Moto_PwmMax;
        Moto_X_Negative   = Moto_PwmMax;
        Moto_Y_Positive   = Moto_PwmMax;
        Moto_Y_Negative   = Moto_PwmMax;
        MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
        delay_ms(2000); 
        Moto_X_Positive   = Moto_PwmMin;
        Moto_X_Negative   = Moto_PwmMin;
        Moto_Y_Positive   = Moto_PwmMin;
        Moto_Y_Negative   = Moto_PwmMin;
        MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
        delay_ms(5000);    
}
void DrYL_Motor_Stop(void)
{
	 Moto_X_Positive   = Moto_PwmMin;
        Moto_X_Negative   = Moto_PwmMin;
        Moto_Y_Positive   = Moto_PwmMin;
        Moto_Y_Negative   = Moto_PwmMin;
	 MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
}

void DrYL_Moto_Start(void)
{
	 Moto_X_Positive   = (Moto_PwmMax-Moto_PwmMin)/4+Moto_PwmMin;
        Moto_X_Negative  = (Moto_PwmMax-Moto_PwmMin)/4+Moto_PwmMin;
        Moto_Y_Positive    = (Moto_PwmMax-Moto_PwmMin)/4+Moto_PwmMin;
        Moto_Y_Negative  = (Moto_PwmMax-Moto_PwmMin)/4+Moto_PwmMin;
	 MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
}
void DrYL_Moto_Top(void)
{
	 Moto_X_Positive    += 10;
        Moto_X_Negative   += 10;
        Moto_Y_Positive     += 10;
        Moto_Y_Negative   += 10;
	
}
void DrYL_Moto_Down(void)
{
	 Moto_X_Positive    -= 10;
        Moto_X_Negative   -= 10;
        Moto_Y_Positive     -= 10;
        Moto_Y_Negative   -= 10;
}
void DrYL_Moto_Left(void)
{

}
void DrYL_Moto_Right(void)
{
	
}



