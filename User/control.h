#ifndef __MY_CONTROL_H__
#define __MY_CONTROL_H_

#define PID_DEAD_AREA 2


typedef struct PID
{
    float SetPoint; //�趨Ŀ�� Desired Value   
    float SumError; //����ۼ�    
    float Proportion; //�������� Proportional Const    
    float Integral; //���ֳ��� Integral Const  
    float Derivative; //΢�ֳ��� Derivative Const   
    float LastError; //Error[-1] 
    float PrevError; //Error[-2]
}PID;

extern void DrYL_IncPIDInit(PID *sptr,const float kp,
                     const float ki,const float kd);
extern u8 DrYL_PID_Control_pitch_roll(void);

//int DrYL_IncPID_Calc(PID *sptr,const float measured,float expect);

extern PID pitch_pid, roll_pid;


//extern void DrYL_IncPIDInit(PID sptr);
extern u16 dmp_times;
#define Moto_PwmMin_Debug 999
#define Moto_PwmMax_Debug 1450  //45%

#endif