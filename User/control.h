#ifndef __MY_CONTROL_H__
#define __MY_CONTROL_H_


typedef struct PID
{
    float SetPoint; //设定目标 Desired Value   
    float SumError; //误差累计    
    int Proportion; //比例常数 Proportional Const    
    int Integral; //积分常数 Integral Const  
    int Derivative; //微分常数 Derivative Const   
    float LastError; //Error[-1] 
    float PrevError; //Error[-2]
}PID;

extern void DrYL_IncPIDInit(PID *sptr);
extern u8 DrYL_PID_Control_pitch_roll(void);



extern PID pitch_pid, roll_pid;


//extern void DrYL_IncPIDInit(PID sptr);
extern u16 dmp_times;
#define Moto_PwmMin_Debug 999
#define Moto_PwmMax_Debug 1450  //45%

#endif