

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "includes.h"

int16_t duty=1000;
void delay(uint16_t t)
{
    while(t--);
}

/**************************************/
/*unsigned long sensor_timestamp1;
short gyro1[3], accel1[3], sensors1;
unsigned char more1;
long quat1[4];
float Yaw=0.00;
float Roll,Pitch;
int a,b,c;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
*/
u16 k1=0;
u16 k2=0;
u8 i=0;
float z_position=0;
u8 quad_up_flag=0;
u8 quad_down_flag=0;
/********************************/
int xxx=0,yyy=0,zzz=0;
int main(void)
{
	
        SystemInit(); //system clock
	TIM4_INIT(); //time4 init
          
	
        DrYL_TIME2_NVIC_Configutation(); // timer2 interupt configure 
        TIM2_INIT(); //delay init
        
        STM32_PWM_INIT();  //PWN init
        
        ANBT_I2C_Configuration();  //i2c init
         
        AnBT_DMP_MPU6050_Init();  // MPU6050 ��ʼ��
        
        ANBT_HMC5883L_MAG_Init_FUN(); //HCM5883 ��ʼ��
        
      /******************����3��ʼ��*********rs232******************/
        DrYL_Uart3_RCC_Configuration();
        AnBT_UART3_GPIO_Configuration();
        AnBT_UART3_Configuration();
        AnBT_UART3_NVIC_Configuration();
       /*****************************************************/ 
        
       /*****************����2��ʼ��****************
        DrYL_Uart2_RCC_Configuration();
        AnBT_UART2_GPIO_Configuration();
        AnBT_UART2_NVIC_Configuration();
        AnBT_UART2Configuration();  
       *******************************************/ 
      //  Gyro_Correct();
        
        DrYL_Motor_Driver_init();  // �����ʼ��     
       // DrYL_IncPIDInit(&pitch_pid);
       // DrYL_IncPIDInit(&roll_pid);
        //run_self_test();
        Moto_X_Positive=Moto_PwmMin;
        Moto_X_Negative=Moto_PwmMin;
        Moto_Y_Positive=Moto_PwmMin;
        Moto_Y_Negative=Moto_PwmMin;
        
        DrYL_Send_Moto_PWM(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);//����Ϊ��С����

	while (1)
	{
         
            //DrYL_Read_MPU6050_GYRO(&xxx,&yyy,&zzz);
             if(StartFlag==1)
             {
                DrYL_PID_Control_pitch_roll();
             }
             DrYL_GetCommand();
             //DrYL_Read_MPU6050_GYRO();
             if(MPU6050_Tim_1ms>20)
             {
        	MPU6050_Tim_1ms=0;
                DrYLSendOneFrameData(); //���������������
             }                 
         }
  
 // return 0;
}


