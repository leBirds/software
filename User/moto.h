#ifndef _MY_MOTO_H__
#define _MY_MOTO_H__


extern void STM32_PWM_INIT(void);
extern void MotoPWMControl(unsigned int moto_x_p,unsigned int moto_x_n,unsigned int moto_y_p,unsigned int moto_y_n);
extern void DrYL_Motor_Driver_init(void);
extern void DrYL_Motor_Stop(void);
extern void DrYL_Moto_Start(void);
extern void DrYL_Moto_Top(void);
extern void DrYL_Moto_Down(void);
extern u16 Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative;

#define Moto_PwmMax	1999    //��������23999,Ϊ�˱������������50%����
#define Moto_PwmMin     999    //ֹͣ����Ϊ11999

#endif