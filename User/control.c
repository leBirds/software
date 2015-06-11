#include "includes.h"

#define q30  1073741824.0f
#define DATA_LENGTH 1
#define FILTER_LENGTH 5
//存取原始数据
/*
float yaw_table[DATA_LENGTH]={0};
float roll_table[DATA_LENGTH]={0};
float pitch_table[DATA_LENGTH]={0};

float yaw_table_d[DATA_LENGTH]={0};
float roll_table_d[DATA_LENGTH]={0};
float pitch_table_d[DATA_LENGTH]={0};
//存取滤波完了以后的数据
float yaw_table_filtered[DATA_LENGTH]={0};
float roll_table_filtered[DATA_LENGTH]={0};
float pitch_table_filtered[DATA_LENGTH]={0};
float gyro_pitch_table[DATA_LENGTH]={0};
float gyro_roll_table[DATA_LENGTH]={0};

*/
//static PID *sptr = &sPID;
 
/*====================================================================================================
Initialize PID Structure PID参数初始化
=====================================================================================================*/
 
void DrYL_IncPIDInit(PID *sptr,const float kp,
                     const float ki,const float kd)
{
    sptr->Proportion = kp; //比例常数 Proportional Const
    sptr->Integral = ki; //积分常数Integral Const
    sptr->Derivative = kd; //微分常数 Derivative Const
    //sptr->SetPoint = desired; 
}


 
/*====================================================================================================
增量式PID计算部分
======================================================================================================*/
 
int DrYL_IncPID_Calc(PID *sptr,const float measured,float expect)
{ 
    float iError; //当前误差   
    int   iIncpid;
    iError = (sptr->SetPoint - measured); //增量计算 
    
    iIncpid = (int)(sptr->Proportion * (iError-sptr->LastError) //E[k]项
              + sptr->Integral * iError //E[k－1]项
              + sptr->Derivative * (iError-2*sptr->LastError+sptr->PrevError)); //E[k－2]项
   
    //存储误差，用于下次计算
    
    sptr->PrevError = sptr->LastError;
    
    sptr->LastError = iError;
    
    //返回增量值
    return(iIncpid);
}
// 位置式PID
int DrYL_DoubleHoopPID_Calc(PID *sptr,const float measured,float expect,float gyro)
{
     float kp=5.0;float rate_error,dError;
     float target_rate;
     float angel_difference; 
     int iError;
     angel_difference= (expect - measured); // 角度差
     if((angel_difference<PID_DEAD_AREA)&&(angel_difference>-PID_DEAD_AREA))
     {
        angel_difference=0;
     }
     if((angel_difference*kp) > ANGEL_LIMIT)
     {
        angel_difference = ANGEL_LIMIT;
     }
     if((angel_difference*kp) < -ANGEL_LIMIT)
     {
        angel_difference = -ANGEL_LIMIT;
     }
     
     target_rate=angel_difference*kp;
     //if(target_rate>30)
     
     rate_error = target_rate+gyro;   //P项
     sptr->SumError+=rate_error;      //  I项
     if(sptr->SumError > I_LIMIT) // I项限幅
     {
        sptr->SumError = I_LIMIT;
     }
     if(sptr->SumError < -I_LIMIT)
     {
        sptr->SumError = -I_LIMIT;
     }
     dError = (rate_error-sptr->LastError);
     if(DrYL_Abs(angel_difference)<D_AREA)
     {
        sptr->Derivative *= 0.3; 
     }
     if(DrYL_Abs(angel_difference)<PID_DEAD_AREA)
     {      
         iError = (int)(sptr->Proportion*rate_error+
                        sptr->Integral*sptr->SumError+
                        sptr->Derivative*dError);
     }
     if(iError>SUM_LIMIT)
     {
        iError=SUM_LIMIT;
     }
     if(iError<-SUM_LIMIT)
     {
        iError=-SUM_LIMIT;
     }
     
     sptr->PrevError = sptr->LastError;
     sptr->LastError=rate_error; // 更新数据
     
     
     return iError;
}
/***********************************************************************************
*************************************************************************************/
void DrYL_IncPIDInit_new(NewPID *sptr,const float kp,
                     const float ki,const float kd)
{
    sptr->Proportion = kp; //比例常数 Proportional Const
    sptr->Integral = ki; //积分常数Integral Const
    sptr->Derivative = kd; //微分常数 Derivative Const
    //sptr->SetPoint = desired; 
}
int pidUpdate(NewPID* sptr, const float measured,float expect,float gyro_t)
{
      int output;
    //  int lastoutput=0;
    
      sptr->desired=expect;			 				//获取期望角度
    
      sptr->Error = sptr->desired - measured;	 	  //偏差：期望-测量值
      if(DrYL_Abs(sptr->Error)<PID_DEAD_AREA)
      {
          sptr->Error=0;   
      }
      sptr->SumError += sptr->Error * IMU_UPDATE_DT;	  //偏差积分
     
      if (sptr->SumError > I_LIMIT)				  //作积分限制
      {
        sptr->SumError= I_LIMIT;
      }
      else if (sptr->SumError < -I_LIMIT)
      {
        sptr->SumError = -I_LIMIT;
      }				 
    
     // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//微分	 应该可用陀螺仪角速度代替
      sptr->derivError = -gyro_t;
    /*  if(DrYL_Abs(sptr->Error)<D_AREA)
      {
        sptr->Derivative *= 0.3; 
      }*/
      if(DrYL_Abs(sptr->Error)>PID_DEAD_AREA)									//pid死区
      {
            sptr->outP = sptr->Proportion * sptr->Error;								 //方便独立观察
            //if(pid->outP > P_LIMIT) pid->outP = P_LIMIT;
            //else if(pid->outP < -P_LIMIT)pid->outP = -P_LIMIT;
            sptr->outI = sptr->Integral * sptr->SumError*0.1;
            sptr->outD = sptr->Derivative * sptr->derivError;
          
            output =(int)(sptr->outP+sptr->outI+sptr->outD);
      }
      else
      {
           output=sptr->lastoutput;
          // sptr->SumError=0;
      }
      /******输出限幅*******************/
      if(output>SUM_LIMIT)
      {
        output=SUM_LIMIT;
      }
      if(output<-SUM_LIMIT)
      {
        output=-SUM_LIMIT;
      }
      /**************************/
      sptr->LastError = sptr->Error;							 		//更新前一次偏差
      sptr->lastoutput=output;
    
      return output;
}

void DrYL_Set_PID(float  p,float  i,float d,PID *sptr)
{
    	sptr->Proportion=p;
	sptr->Integral=	i;
	sptr->Derivative=d;
}
void DrYL_Set_PID_New(float  p,float  i,float d,NewPID *sptr)
{
    	sptr->Proportion=p;
	sptr->Integral=	i;
	sptr->Derivative=d;
}
u16  dmp_times=0;
/********************************************************************/
/*******************************/
//u8 hc5883_data[6];
//int x,y,z;
//float angel;
float gyroPID[3];
//float  gyro_times=0;
/**************************/
short gyro1[3];
u8 DrYL_Get_Gesture(float *yaw,float *pitch, float *roll, float *accel)
{
       short accel1[3], sensors1;
       
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
           //norm=sqrt(quat1[0]*quat1[0] + quat1[1]*quat1[1] + quat1[2]*quat1[2] + quat1[3]*quat1[3]);
           q0=quat1[0] / q30;
           q1=quat1[1] / q30;
           q2=quat1[2] / q30;
           q3=quat1[3] / q30;
           *pitch   =   asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
           *roll    =   atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
           //*yaw     =   atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
       /*     ANBT_HMC5883L_MAG_Read_Data_FUN(hc5883_data);
            x=(int)(hc5883_data[0]<<8|hc5883_data[1]);
            z=(int)(hc5883_data[2]<<8|hc5883_data[3]);
            y=(int)(hc5883_data[4]<<8|hc5883_data[5]);
            if(x>0x7fff)x-=0xffff;
            if(y>0x7fff)y-=0xffff;
            *yaw=atan2((float)y,(float)x)*57.3+180;
           *********************************************************/
          //  加速度处理
      /*     accel[0]=accel1[0]/32768.0*2;
           accel[1]=accel1[1]/32768.0*2;
           accel[2]=accel1[2]/32768.0*2;*/
           gyroPID[0]=(float)(gyro1[0]/16.4);
           gyroPID[1]=(float)(gyro1[1]/16.4);
           gyroPID[2]=(float)(gyro1[2]/16.4);
           /*if(gyroPID[0]>gyro_times)
           {
              gyro_times=gyroPID[0];
           }*/
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
float DrYL_Shift_Averenge_Filter(float *sen,u8 len)
{
    u8 i;
    float sum=0;
    for(i=0;i<len;i++)
    {
      sum+=sen[i];
    }
    return (sum/len);
}
float DrYL_Filter(float d_n,float d_o)
{
    /*if(DrYL_Abs(d_n-d_o)>10)return d_o;
    else return d_n;*/
   return ((d_n+d_o)/2.0); 
}

/***********************************************************************/
float Yaw=0.00;
float Roll,Pitch;
float yaw_filter=0,roll_filter=0,pitch_filter=0;
float gyro_pitch_filter=0,gyro_roll_filter=0;
//int a,b,c;
/*
Model Yaw_M={0.0,0.0};
Model Roll_M={0.0,0.0};
Model Pitch_M={0.0,0.0};
Model gyro_x_M={0.0,0.0};
Model gyro_y_M={0.0,0.0};
Model gyro_z_M={0.0,0.0};*/

//PID pitch_pid={0,0,0.1,0,0,0,0};
//PID roll_pid ={0,0,0.1,0,0,0,0};
/*******************************/
NewPID pitch_pid_new={0, 2.0,0.7,6.0,  0,0,0,0, 0,0,0, 0};
NewPID roll_pid_new= {0, 2.0,0.7,-6.0,  0,0,0,0, 0,0,0, 0};
/******************************/
//u8   pitch_pid_times=0;
//u8   roll_pid_times=0;

float accel_actual[3];
/*****************debug wariable********/
//u8 accel_times=0;
//float pitch_max=0;
//float roll_max=0;
/**********************************************************/
int roll_pid_result=0,pitch_pid_result=0,yaw_pid_result=0;//PID计算出的结果

//u8 get_sensor_times=0;
u16 MotoGiven=999;
#define PIDMIX(X,Y,Z) MotoGiven + (pitch_pid_result*(X)) + (roll_pid_result*(Y)) + (yaw_pid_result*(Z))	
u8 DrYL_PID_Control_pitch_roll(void)             
{
        if(TIM2_IRQCNT>4)
        {
            TIM2_IRQCNT=0; 
             
            
            DrYL_Get_Gesture(&Yaw,&Pitch,&Roll,accel_actual); //获取姿态和加速度
           // if(Roll>roll_max)roll_max=Roll;
            //更新数据
           /* roll_table[DATA_LENGTH-1]=Roll;
            pitch_table[DATA_LENGTH-1]=Pitch;
            gyro_pitch_table[DATA_LENGTH-1]=gyroPID[0];
            gyro_roll_table[DATA_LENGTH-1]=gyroPID[1];
            */
          //  DrYL_Updata_Sensor(roll_table,DATA_LENGTH,Roll);
           // DrYL_Updata_Sensor(pitch_table,DATA_LENGTH,Pitch); 
           // DrYL_Updata_Sensor(gyro_pitch_table,DATA_LENGTH,gyroPID[0]);
           // DrYL_Updata_Sensor(gyro_roll_table,DATA_LENGTH,gyroPID[1]);
            
           // if(gyroPID[1]>pitch_max)pitch_max=gyroPID[1];
            
           // roll_filter=DrYL_Shift_Averenge_Filter(roll_table,DATA_LENGTH);
           // pitch_filter=DrYL_Shift_Averenge_Filter(pitch_table,DATA_LENGTH);
           // gyro_pitch_filter=DrYL_Shift_Averenge_Filter(gyro_pitch_table,DATA_LENGTH);
           // gyro_roll_filter=DrYL_Shift_Averenge_Filter(gyro_roll_table,DATA_LENGTH);
           
            
            DrYL_IncPIDInit_new(&pitch_pid_new,pitch_pid_new.Proportion,pitch_pid_new.Integral,pitch_pid_new.Derivative);
            DrYL_IncPIDInit_new(&roll_pid_new,roll_pid_new.Proportion,roll_pid_new.Integral,roll_pid_new.Derivative);

            pitch_pid_result=pidUpdate(&pitch_pid_new,Pitch,0,gyroPID[0]); // 计算位置式参数
            roll_pid_result=pidUpdate(&roll_pid_new,Roll,0,gyroPID[1]); //
            
            if(MotoGiven<1150) //预防刚起步就有大的动作
            {
                pitch_pid_new.SumError=0;
                roll_pid_new.SumError=0;
            }
            
            Moto_X_Negative =PIDMIX(-1,0,0);// MotoGiven-pitch_pid_result;
            Moto_X_Positive =PIDMIX(+1,0,0);//MotoGiven+pitch_pid_result;

            Moto_Y_Negative=PIDMIX(0,-1,0);//MotoGiven-roll_pid_result; // 
            Moto_Y_Positive=PIDMIX(0,+1,0);//MotoGiven+roll_pid_result;// 位置式PID

  
            /***********************   限幅    ***************************/
            if(Moto_X_Positive   > Moto_PwmMax_Debug)	    Moto_X_Positive    = Moto_PwmMax_Debug;
            if(Moto_X_Negative   > Moto_PwmMax_Debug)	    Moto_X_Negative    = Moto_PwmMax_Debug;
            if(Moto_Y_Positive   > Moto_PwmMax_Debug)	    Moto_Y_Positive    = Moto_PwmMax_Debug;
            if(Moto_Y_Negative   > Moto_PwmMax_Debug)       Moto_Y_Negative    = Moto_PwmMax_Debug;
            if(Moto_X_Positive   <=Moto_PwmMin_Debug)       Moto_X_Positive    = Moto_PwmMin_Debug;
            if(Moto_X_Negative   <=Moto_PwmMin_Debug)       Moto_X_Negative    = Moto_PwmMin_Debug;
            if(Moto_Y_Positive   <=Moto_PwmMin_Debug)       Moto_Y_Positive    = Moto_PwmMin_Debug;
            if(Moto_Y_Negative   <=Moto_PwmMin_Debug)       Moto_Y_Negative    = Moto_PwmMin_Debug;
            /***************************************************************************************/
            MotoPWMControl(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);
           
           // DrYL_Updata_Sensor(roll_table_d,DATA_LENGTH,roll_filter);
           // DrYL_Updata_Sensor(pitch_table_d,DATA_LENGTH,pitch_filter); 
           // DrYL_Updata_Sensor(gyro_pitch_table_d,DATA_LENGTH,gyro_pitch_filter);
            //DrYL_Updata_Sensor(gyro_roll_table_d,DATA_LENGTH,gyro_roll_filte);
             
            
            
           /* gyro_x_M.now=gyroPID[0];
            gyro_y_M.now=gyroPID[1];
            gyro_z_M.now=gyroPID[2];
           
            ***********************滤波 两次之间的差值不能大于 1 ***************************
            Yaw_M.old=DrYL_Filter(Yaw_M.now,Yaw_M.old);
            Pitch_M.old=DrYL_Filter(Pitch_M.now,Pitch_M.old);
            Roll_M.old=DrYL_Filter(Roll_M.now,Roll_M.old);
            gyro_x_M.old=DrYL_Filter(gyro_x_M.now,gyro_x_M.old);
            gyro_y_M.old=DrYL_Filter(gyro_y_M.now,gyro_y_M.old);
            gyro_z_M.old=DrYL_Filter(gyro_z_M.now,gyro_z_M.old);
            
            ********************************************************/
           
                      //DrYL_Send_Moto_PWM(Moto_X_Positive,Moto_X_Negative,Moto_Y_Positive,Moto_Y_Negative);

        }
        
        if(time_1ms_cnt>10)
        {
            time_1ms_cnt=0; 
            
        }
	
        return 0;
}


//float aa=0,bb=0,cc=0;
void DrYL_Read_MPU6050_GYRO(int *gyro_x,int *gyro_y,int *gyro_z)
{
  u8 gyro_data[6]={0};
  AnBT_DMP_I2C_Read(0x68, GYRO_XOUT_L ,  1 , &gyro_data[0]);
  AnBT_DMP_I2C_Read(0x68, GYRO_XOUT_H ,  1 , &gyro_data[1]);
  AnBT_DMP_I2C_Read(0x68, GYRO_YOUT_L ,  1 , &gyro_data[2]);
  AnBT_DMP_I2C_Read(0x68, GYRO_YOUT_H ,  1 , &gyro_data[3]);
  AnBT_DMP_I2C_Read(0x68, GYRO_ZOUT_L ,  1 , &gyro_data[4]);
  AnBT_DMP_I2C_Read(0x68, GYRO_ZOUT_H ,  1 , &gyro_data[5]);
 // *gyro_x=gyro_data[1];
 // *gyro_y=gyro_data[3];
 // *gyro_z=gyro_data[5];
  *gyro_x= (int)(gyro_data[1]<<8|gyro_data[0]);
  *gyro_y= (int)(gyro_data[3]<<8|gyro_data[2]);
  *gyro_z= (int)(gyro_data[5]<<8|gyro_data[4]); 
}
int Gyrooffset_X=0,Gyrooffset_Y=0,Gyrooffset_Z=0;
void Gyro_Correct(void)
{
	unsigned char i=0;
	unsigned char numGyro=200;

	int Gyrox=0;
	int Gyroy=0;
	int Gyroz=0;						  //陀螺仪校正中间变量
        int gx=0,gy=0,gz=0;
	for(i=0;i<numGyro;i++)
	{
		DrYL_Read_MPU6050_GYRO(&gx,&gy,&gz);
		Gyrox+=gx;
		Gyroy+=gy;
		Gyroz+=gz;
		delay_ms(2);
	}	

	Gyrooffset_X= Gyrox/numGyro;					   
	Gyrooffset_Y= Gyroy/numGyro;
	Gyrooffset_Z= Gyroz/numGyro;
}