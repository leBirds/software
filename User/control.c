#include "includes.h"

#define q30  1073741824.0f
#define DATA_LENGTH 20
#define FILTER_LENGTH 5
//存取原始数据
float yaw_table[DATA_LENGTH]={0};
float roll_table[DATA_LENGTH]={0};
float pitch_table[DATA_LENGTH]={0};
//存取滤波完了以后的数据
float yaw_table_filtered[DATA_LENGTH]={0};
float roll_table_filtered[DATA_LENGTH]={0};
float pitch_table_filtered[DATA_LENGTH]={0};


//static PID *sptr = &sPID;
 
/*====================================================================================================
Initialize PID Structure PID参数初始化
=====================================================================================================*/
 
void DrYL_IncPIDInit(PID *sptr,const float kp,
                     const float ki,const float kd)
{
    sptr->SumError = 0; 
    sptr->LastError = 0; //Error[-1]   
    sptr->PrevError = 0; //Error[-2]
    sptr->Proportion = kp; //比例常数 Proportional Const
    sptr->Integral = ki; //积分常数Integral Const
    sptr->Derivative = kd; //微分常数 Derivative Const
    //sptr->SetPoint = desired; 
}
 
/*====================================================================================================
增量式PID计算部分
=====================================================================================================*/
 
int DrYL_IncPID_Calc(PID *sptr,const float measured,float expect)
{ 
    int iError, iIncpid; //当前误差   
    iError = (int)(sptr->SetPoint - measured); //增量计算 
    
    iIncpid = (int)(sptr->Proportion * (iError-sptr->LastError) //E[k]项
              + sptr->Integral * iError //E[k－1]项
              + sptr->Derivative * (iError-2*sptr->LastError+sptr->PrevError)); //E[k－2]项
   
    //存储误差，用于下次计算
    sptr->PrevError = sptr->LastError;
    
    sptr->LastError = iError;
    //返回增量值
    return(iIncpid);
}
/*
void DrYL_Set_PID(int p,int i,int d,PID *sptr)
{
    	sptr->Proportion=p;
	sptr->Integral=	i;
	sptr->Derivative=d;
}*/
u16  dmp_times=0;
/********************************************************************/
/*******************************/
char hc5883_data[6];
int x,y,z;
float angel;
/**************************/
u8 DrYL_Get_Gesture(float *yaw,float *pitch, float *roll, float *accel)
{
       short gyro1[3], accel1[3], sensors1;
       unsigned long sensor_timestamp1;
       long quat1[4];
       unsigned char more1;
       float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
       if(dmp_read_fifo(gyro1, accel1, quat1, &sensor_timestamp1, &sensors1,&more1)==0)
       {
          dmp_times++;
       }
       if(sensors1 & INV_WXYZ_QUAT )
       {
           q0=quat1[0] / q30;
           q1=quat1[1] / q30;
           q2=quat1[2] / q30;
           q3=quat1[3] / q30;
           *pitch   =   asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
           *roll    =   atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
           //*yaw     =   atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
            ANBT_HMC5883L_MAG_Read_Data_FUN(hc5883_data);
            x=(int)(hc5883_data[0]<<8|hc5883_data[1]);
            z=(int)(hc5883_data[2]<<8|hc5883_data[3]);
            y=(int)(hc5883_data[4]<<8|hc5883_data[5]);
            if(x>0x7fff)x-=0xffff;
            if(y>0x7fff)y-=0xffff;
            *yaw=atan2((float)y,(float)x)*57.3+180;
           /*********************************************************/
          //  加速度处理
           accel[0]=accel1[0]/32768.0*2;
           accel[1]=accel1[1]/32768.0*2;
           accel[2]=accel1[2]/32768.0*2;
           //if(accel_actual[2]>1.5){accel_times++;}
       }
       return 0;
}
/**********************************************************************

********************************************************************/
//  滤波
void DrYL_Sensor_Filter(float *y_table,float *p_table, float *r_table, u8 num,float *y,float *p, float *r) // 均值滤波
{
    float y_tem=0;
    float p_tem=0;
    float r_tem=0;
    u8 i;
    for (i=0;i<num;i++)
    {
        y_tem+=y_table[i];
        p_tem+=p_table[i];
        r_tem+=r_table[i];
    }
    *y=y_tem/num;
    *p=p_tem/num;
    *r=r_tem/num;
}
// iFiterLen  数据长度
float  GetMedianNum(float * bArray, u8 iFilterLen)   //获得中值
{
	int i,j;// 循环变量
	float bTemp;
	
	// 用冒泡法对数组进行排序
	for (j = 0; j < iFilterLen - 1; j ++)
	{
		for (i = 0; i < iFilterLen - j - 1; i ++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// 互换
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}
	
	// 计算中值
	if ((iFilterLen & 1) > 0)
	{
		// 数组有奇数个元素，返回中间一个元素
		bTemp = bArray[(iFilterLen + 1) / 2];
	}
	else
	{
		// 数组有偶数个元素，返回中间两个元素平均值
		bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;
	}

	return bTemp;
}
//中值滤波 
//长度为: 3
void DrYL_Median_Filter(float *bArry,  float *bArry_new, u8 iDataLen ) 
{
    	u8 i;
	float temp[FILTER_LENGTH];
	for(i=0;i<iDataLen;i++)
	{
		bArry_new[i]=bArry[i];
	}
	for(i=0;i<iDataLen-FILTER_LENGTH;i++)
	{
		temp[0]=bArry_new[i];
		temp[1]=bArry_new[i+1];
		temp[2]=bArry_new[i+2];
                temp[3]=bArry_new[i+3];
                temp[4]=bArry_new[i+4];
		bArry_new[i+2]=GetMedianNum(temp,5);
	}
}

/******************************************************************
//更新传感器的数据
**/
void DrYL_Updata_Sensor(float *sen, u8 len,float sen_new)
{
    u8 i;
    for(i=0;i<(len-1);i++)
    {
        sen[i]=sen[i+1];
    }
    sen[len-1]=sen_new;
    
}


/***********************************************************************/
float Yaw=0.00;
float Roll,Pitch;
int a,b,c;

PID pitch_pid, roll_pid;
u8   pitch_pid_times=0;
u8   roll_pid_times=0;

float accel_actual[3];
/*****************debug wariable********/
u8 accel_times=0;

/*************************/
int roll_pid_result,pitch_pid_result;
u8 get_sensor_times=0;

u8 DrYL_PID_Control_pitch_roll(void)             
{
        
        if(MPU6050_Tim_1ms>10)
        {
        	MPU6050_Tim_1ms=0;
        }
        
        if(TIM2_IRQCNT>10)
        {
            TIM2_IRQCNT=0; 
             
            
            DrYL_Get_Gesture(&Yaw,&Pitch,&Roll,accel_actual); //获取姿态和加速度
        
            DrYL_Send_Yaw(Yaw);
            DrYL_Send_Pitch(Pitch);
            DrYL_Send_Roll(Roll);
            
            DrYL_Send_Moto(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
         
           // DrYL_Set_PID(1,0,0,&pitch_pid);//设置pitch  pid参数
           // DrYL_Set_PID(1,0,0,&roll_pid);//设置roll  pid 参数
            DrYL_IncPIDInit(&pitch_pid,2,0,0);
            DrYL_IncPIDInit(&roll_pid,2,0,0);
            pitch_pid_result=DrYL_IncPID_Calc(&pitch_pid,Pitch,0); // 计算增量型参数
            roll_pid_result=DrYL_IncPID_Calc(&roll_pid,Roll,0); //
            // 一个调节周期之内不会让它一直上升,只调节一对电机，另一对电机作为参考 
            if((pitch_pid.LastError<PID_DEAD_AREA )&&(pitch_pid.LastError>-PID_DEAD_AREA ))// 死区
            {
                    
            }
            else
            {
                   Moto_X_Negative+=pitch_pid_result;
                   Moto_X_Positive-=pitch_pid_result;
            }
            if((roll_pid.LastError<PID_DEAD_AREA )&&(roll_pid.LastError>-PID_DEAD_AREA )) //死区
            {
                     
            }
            else
            {  
              
                    Moto_Y_Negative+=roll_pid_result;
                    Moto_Y_Positive-=roll_pid_result;
            }	
            
            if(Moto_X_Positive   > Moto_PwmMax_Debug)	    Moto_X_Positive    = Moto_PwmMax_Debug;
            if(Moto_X_Negative   > Moto_PwmMax_Debug)	    Moto_X_Negative    = Moto_PwmMax_Debug;
            if(Moto_Y_Positive   > Moto_PwmMax_Debug)	    Moto_Y_Positive    = Moto_PwmMax_Debug;
            if(Moto_Y_Negative   > Moto_PwmMax_Debug)       Moto_Y_Negative    = Moto_PwmMax_Debug;
            if(Moto_X_Positive   <=Moto_PwmMin_Debug)       Moto_X_Positive    = Moto_PwmMin_Debug;
            if(Moto_X_Negative   <=Moto_PwmMin_Debug)       Moto_X_Negative    = Moto_PwmMin_Debug;
            if(Moto_Y_Positive   <=Moto_PwmMin_Debug)       Moto_Y_Positive    = Moto_PwmMin_Debug;
            if(Moto_Y_Negative   <=Moto_PwmMin_Debug)       Moto_Y_Negative    = Moto_PwmMin_Debug;
            MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
                      //DrYL_Send_Moto_PWM(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);

        }
	
        return 0;
}
