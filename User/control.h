#ifndef __MY_CONTROL_H__
#define __MY_CONTROL_H_


typedef struct PID
{
    float SetPoint; //�趨Ŀ�� Desired Value   
    float SumError; //����ۼ�    
    int Proportion; //�������� Proportional Const    
    int Integral; //���ֳ��� Integral Const  
    int Derivative; //΢�ֳ��� Derivative Const   
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