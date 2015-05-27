

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

int main(void)
{
	
        SystemInit(); //system clock
	TIM4_INIT(); //time4 init
          
	
        DrYL_TIME2_NVIC_Configutation(); // interupt configure 
        TIM2_INIT(); //delay init
        
        STM32_PWM_INIT();  //PWN init
        
        ANBT_I2C_Configuration();  //i2c init
         
        AnBT_DMP_MPU6050_Init();  // MPU6050 初始化
        
        ANBT_HMC5883L_MAG_Init_FUN(); //HCM5883 初始化
        
      /******************串口3初始化*********rs232******************/
        DrYL_Uart3_RCC_Configuration();
        AnBT_UART3_GPIO_Configuration();
        AnBT_UART3_Configuration();
        AnBT_UART3_NVIC_Configuration();
       /*****************************************************/ 
        
       /*****************串口2初始化****************/
        DrYL_Uart2_RCC_Configuration();
        AnBT_UART2_GPIO_Configuration();
        AnBT_UART2_NVIC_Configuration();
        AnBT_UART2Configuration();  
       /*******************************************/ 
        
        
       DrYL_Motor_Driver_init();  // 电调初始化     
       // DrYL_IncPIDInit(&pitch_pid);
       // DrYL_IncPIDInit(&roll_pid);
        //run_self_test();
        Moto_X_Positive=Moto_PwmMin;
        Moto_X_Negative=Moto_PwmMin;
        Moto_Y_Positive=Moto_PwmMin;
        Moto_Y_Negative=Moto_PwmMin;
        
        DrYL_Send_Moto_PWM(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);//设置为最小油门
    /*    for(i = 0; i <40;i++)
        {
             Moto_X_Positive+=10;
             Moto_Y_Positive+=10;
             Moto_Y_Negative+=10;
             DrYL_Send_Moto_PWM(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
             delay_ms(100);
        }*/
       // AnBT_Uart3_Send_Char('.');
	while (1)
	{
             if(StartFlag==1)
             {
                DrYL_PID_Control_pitch_roll();
             }
             DrYL_GetCommand();
             if(MPU6050_Tim_1ms>1)
             {
        	MPU6050_Tim_1ms=0;
                DrYLSendOneFrameData(); //解析命令，发送数据
             }
            /* if(time_1ms_cnt>5000)
             { 
                    time_1ms_cnt=0;
                    
		    if((quad_up_flag==0 && quad_down_flag ==0)||(quad_up_flag==1 && quad_down_flag ==0))
		    {
	                 Moto_X_Positive+=10;
	                 Moto_X_Negative+=10;
	                 Moto_Y_Positive+=10;
	                 Moto_Y_Negative+=10;
                          if((Moto_X_Positive>Moto_PwmMax_Debug)||(Moto_X_Negative>Moto_PwmMax_Debug)||
                            (Moto_Y_Positive>Moto_PwmMax_Debug)||(Moto_Y_Negative>Moto_PwmMax_Debug))
                          {
                                  quad_up_flag=0;
                                  quad_down_flag =1 ;
                          }
                          else
                          {
                                quad_up_flag =1;
                                quad_down_flag =0;
                          }
                          
		    }
		    else if(quad_down_flag==1 && quad_up_flag ==0)
		    {
		    	 Moto_X_Positive-=10;
	                 Moto_X_Negative-=10;
	                 Moto_Y_Positive-=10;
	                 Moto_Y_Negative-=10;
                         if((Moto_X_Negative<=Moto_PwmMin_Debug)||(Moto_X_Positive <= Moto_PwmMin_Debug)||
                              (Moto_Y_Negative<=Moto_PwmMin_Debug)||(Moto_Y_Positive <= Moto_PwmMin_Debug))
                         {
                                 DrYL_Motor_Stop();
                         }
		    }
		    
             
		
      
             }
             
             */
                          
         }
  
 // return 0;
}


