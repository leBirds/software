#ifndef __MY_TIMER_H__
#define __MY_TIMER_H__

extern void delay_ms(uint16_t nms);
extern void delay_us(uint16_t nus);
extern void TIM4_INIT(void);
extern void TIM2_INIT(void);
extern void DrYL_TIME2_NVIC_Configutation(void);

extern void TIM2_IRQ(void);
extern volatile uint32_t	TIM2_IRQCNT;
extern volatile uint32_t	time_1ms_cnt; 	
extern volatile uint32_t        MPU6050_Tim_1ms;
#endif