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
	 cmdStart               =	 	0x30,  //  ��ʼ
	 cmdStop                =	 	0x31,  //  ����
	 cmdTop               	=  		0x32,  //  ����
	 cmdDown                =  		0x33,  // �½�
	 cmdLeft              	=  		0x34,  // ��
	 cmdRight            	=  		0x35,  // ��
         cmdForward	        =	 	0x36,  // ǰ
         cmdBack	      	=	 	0x37,  // ��
         
	 cmdConfig	      	=  		0x3a,  
       /****************Pitch pid ��**************************************************/  
         cmdPID_Pitch_P_INC           =               0x3d,  // ����pid_p 
         cmdPID_Pitch_I_INC           =               0x3e,  // ����pid_i
         cmdPID_Pitch_D_INC           =               0x3f,  // ����pid_d
        /*****************Pitch pid ��************************************************/
         cmdPID_Pitch_P_DEC           =               0x40,  // ����pid_p 
         cmdPID_Pitch_I_DEC           =               0x41,  // ����pid_i
         cmdPID_Pitch_D_DEC           =               0x42,  // ����pid_d
         
         
          /****************Roll pid ��**************************************************/  
         cmdPID_Roll_P_INC           =               0x43,  // ����pid_p 
         cmdPID_Roll_I_INC           =               0x44,  // ����pid_i
         cmdPID_Roll_D_INC           =               0x45,  // ����pid_d
        /*****************Roll pid ��************************************************/
         cmdPID_Roll_P_DEC           =               0x46,  // ����pid_p 
         cmdPID_Roll_I_DEC           =               0x47,  // ����pid_i
         cmdPID_Roll_D_DEC           =               0x48,  // ����pid_d
	 	  
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