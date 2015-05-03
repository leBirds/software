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
	 cmdStart             =	 	0x30,
	 cmdStop              =	 	0x31,
	 cmdTop               	=  		0x32,
	 cmdDown            =  		0x33,
	 cmdLeft              	=  		0x34,
	 cmdRight            	=  		0x35,
         cmdForward	=	 	0x36,
         cmdBack	      	=	 	0x37,
         
	 cmdConfig	      	=  		0x0a,  
	 	  
};
#endif