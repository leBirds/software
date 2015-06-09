#ifndef __MY_CONTROL_H__
#define __MY_CONTROL_H_

#define PID_DEAD_AREA 0.8

#define I_LIMIT   50
#define ANGEL_LIMIT  10
#define SUM_LIMIT   400  // 40%的油门
#define IMU_UPDATE_DT 0.004

#define Moto_PwmMin_Debug 999
#define Moto_PwmMax_Debug 1450  //45%

#define	GYRO_XOUT_H		0x43  //陀螺仪
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

typedef struct PID
{
    float SetPoint; //设定目标 Desired Value   
    float SumError; //误差累计    
    float Proportion; //比例常数 Proportional Const    
    float Integral; //积分常数 Integral Const  
    float Derivative; //微分常数 Derivative Const   
    float LastError; //Error[-1] 
    float PrevError; //Error[-2]
}PID;
typedef struct NewPID
{
      float desired; //设定目标 Desired Value   
      
      float Proportion; //比例常数 Proportional Const    
      float Integral; //积分常数 Integral Const  
      float Derivative; //微分常数 Derivative Const   
      
      float Error;
      float SumError; //误差累计   
      float derivError;
      float LastError; //Error[-1] 
      //float PrevError; //Error[-2]
      float outP;
      float outI;
      float outD;
      int lastoutput;

     
}NewPID;

typedef struct Model
{
      float now;
      float old;
           
}Model;

extern NewPID pitch_pid_new;
extern NewPID roll_pid_new;




//int DrYL_IncPID_Calc(PID *sptr,const float measured,float expect);

extern PID pitch_pid, roll_pid;

extern float Yaw;
extern float Roll,Pitch;
extern float yaw_filter,roll_filter,pitch_filter;

//extern void DrYL_IncPIDInit(PID sptr);
extern u16 dmp_times;
extern u16 MotoGiven;



extern void DrYL_IncPIDInit(PID *sptr,const float kp,
                     const float ki,const float kd);
extern u8 DrYL_PID_Control_pitch_roll(void);

extern void DrYL_Read_MPU6050_GYRO(int *gyro_x,int *gyro_y,int *gyro_z);
extern void Gyro_Correct(void);

#endif