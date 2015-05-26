#ifndef _DRYL_PROTOCOL_H__
#define _DRYL_PROTOCOL_H__



extern u8 DrYL_Send_Moto_PWM(u16 moto1,u16 moto2,u16 moto3,u16 moto4);
extern void AnBT_UART3_Interrupt(void);
extern void DrYL_GetCommand(void);
extern void DrYL_Send_Yaw(float y);
extern void DrYL_Send_Roll(float r);
extern void DrYL_Send_Pitch(float p);
extern  u8  rx_buffer[];  
extern void DrYL_Send_Sensor(float y);
extern void DrYL_Send_Moto(u16 xp ,u16 xn, u16 yp, u16 yn);
enum TCommand
{
	 cmdStart               =	 	0x30,  //  开始
	 cmdStop                =	 	0x31,  //  结束
	 cmdTop               	=  		0x32,  //  上升
	 cmdDown                =  		0x33,  // 下降
	 cmdLeft              	=  		0x34,  // 左
	 cmdRight            	=  		0x35,  // 右
         cmdForward	        =	 	0x36,  // 前
         cmdBack	      	=	 	0x37,  // 后
         
	 cmdConfig	      	=  		0x3a,  
       /****************Pitch pid 升**************************************************/  
         cmdPID_Pitch_P_INC           =               0x3d,  // 调节pid_p 
         cmdPID_Pitch_I_INC           =               0x3e,  // 调节pid_i
         cmdPID_Pitch_D_INC           =               0x3f,  // 调节pid_d
        /*****************Pitch pid 降************************************************/
         cmdPID_Pitch_P_DEC           =               0x40,  // 调节pid_p 
         cmdPID_Pitch_I_DEC           =               0x41,  // 调节pid_i
         cmdPID_Pitch_D_DEC           =               0x42,  // 调节pid_d
         
         
          /****************Roll pid 升**************************************************/  
         cmdPID_Roll_P_INC           =               0x43,  // 调节pid_p 
         cmdPID_Roll_I_INC           =               0x44,  // 调节pid_i
         cmdPID_Roll_D_INC           =               0x45,  // 调节pid_d
        /*****************Roll pid 降************************************************/
         cmdPID_Roll_P_DEC           =               0x46,  // 调节pid_p 
         cmdPID_Roll_I_DEC           =               0x47,  // 调节pid_i
         cmdPID_Roll_D_DEC           =               0x48,  // 调节pid_d
	 	  
};



/**************************************/
extern void DrYL_Pitch_PID_Inc_P(void);
extern void DrYL_Pitch_PID_Inc_I(void);
extern void DrYL_Pitch_PID_Inc_D(void);

extern void DrYL_Pitch_PID_Dec_P(void);
extern void DrYL_Pitch_PID_Dec_I(void);
extern void DrYL_Pitch_PID_Dec_D(void);

extern void DrYL_Roll_PID_Inc_P(void);
extern void DrYL_Roll_PID_Inc_I(void);
extern void DrYL_Roll_PID_Inc_D(void);

extern void DrYL_Roll_PID_Dec_P(void);
extern void DrYL_Roll_PID_Dec_I(void);
extern void DrYL_Roll_PID_Dec_D(void);

extern void DrYLSendOneFrameData(void);
#endif