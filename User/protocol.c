#include "includes.h"

#define STA_START        0        //包头接受状态标志 
#define STA_COMMD        1        //命令接受状态标志
#define STA_END          2        //包尾接收状态标志
u8  rx_buffer[50];           // 缓存
static u8  START_STRING[5] = "Head";      //数据包头      
static u8  END_STRING[5]   = "Tail";      //数据包尾
static u8  *pstart    = START_STRING;   //指向数据头的指针
static u8  *pend      = END_STRING;     //指向数据尾的指针
static u8  *Prx       = rx_buffer;   //指向接收到的数据的指针
u8    index1       = 0;           //索引
u8    index2       = 0;           //索引

u8 DrYL_Send_Moto_PWM(u16 moto1,u16 moto2,u16 moto3,u16 moto4)
{
    
    DrYL_Uart3_Send_Uint(moto1);
    DrYL_Uart3_Send_Uint(moto2);
    DrYL_Uart3_Send_Uint(moto3);
    DrYL_Uart3_Send_Uint(moto4);
    return 0;
}
u8 com_read_flag=0;
void AnBT_UART3_Interrupt(void)
{
	   if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)	//圆点博士:发生的是串口接收中断
	   {
	        u8 temp;
	        static u8 state = STA_START ; 
	        temp=USART_ReceiveData(USART3);
	        USART_ClearFlag(USART3,USART_FLAG_RXNE);	//圆点博士:清除中断标志
	        
	        switch(state) 
	        {     
	             case STA_START:           
	             {
		               if(temp==*pstart) 
		               {
			                  index1++;
			                  pstart++;
			                  if( index1==4)           
			                  {                        
			                     index1 = 0;
			                     pstart = START_STRING;   
			                     state = STA_COMMD;    
			                  }
		               }
		               else 
		               {
			                  index1 = 0;
			                  pstart = START_STRING;         
		               } 
		              break;       
	             }
	            
	             case   STA_COMMD: 
	             {
		                *Prx  = temp;                     
		                state = STA_END;   
				 
		                break;       
	             } 
	             case STA_END: 
	             {
	                
		                if(temp==*pend) 
		                {
			                  index2++;
			                  pend++;
			                  if(index2==4)          
			                  {                        
				                     index1 = 0;
				                     index2 = 0;  // 索引归位
				                     
				                     pend    	= END_STRING;  //  指针归位    
				                     pstart  	= START_STRING; 
				                     Prx       	= rx_buffer;      
				                     state    	= STA_START;     
						     com_read_flag =1; //接收一帧数据完成，标志位置一
			                  }
		                }
		                else 
		                {
			                     index2 = 0;
			                     pend = END_STRING;      
			                     state = STA_START;    
		                }
	                 break;  
	             }    
	         }
	   
	   }
       
}




void DrYL_GetCommand(void)
{
	u8 cmd;
	if(com_read_flag ==1)
	{
		com_read_flag =0;// 标志位清零
		cmd = rx_buffer[0];
		switch(cmd)
		{
			case cmdStart :
				DrYL_Moto_Start();
                               // AnBT_Uart3_Send_Char('1');  // 调试
                               // AnBT_Uart3_Send_Char('2');   // 调试
			break;
			case cmdStop:
				DrYL_Motor_Stop();
			break;
			case cmdTop:
				 DrYL_Moto_Top();
			break;
			case cmdDown:
				DrYL_Moto_Down();
			break;
			case cmdLeft:
				
			break;
			case cmdRight:
				
			break;
                        case cmdConfig:
                            //DrYLSendOneFrameData();
                        break;
                  
                        case cmdPID_Pitch_P_INC :
                           DrYL_Pitch_PID_Inc_P();
			break;
			case cmdPID_Pitch_I_INC:
                          DrYL_Pitch_PID_Inc_I();
			break;
			case cmdPID_Pitch_D_INC:
                          DrYL_Pitch_PID_Inc_D();
			break;
			case cmdPID_Pitch_P_DEC:
                          DrYL_Pitch_PID_Dec_P();
			break;
			case cmdPID_Pitch_I_DEC:
                          DrYL_Pitch_PID_Dec_I();
			break;
			case cmdPID_Pitch_D_DEC:
                          DrYL_Pitch_PID_Dec_D();
			break;
                        
                        case cmdPID_Roll_P_INC :
                          DrYL_Roll_PID_Inc_P();
			break;
			case cmdPID_Roll_I_INC:
                          DrYL_Roll_PID_Inc_I();
			break;
			case cmdPID_Roll_D_INC:
                           DrYL_Roll_PID_Inc_D();
			break;
			case cmdPID_Roll_P_DEC:
                          DrYL_Roll_PID_Dec_P();
			break;
			case cmdPID_Roll_I_DEC:
                          DrYL_Roll_PID_Dec_I();
			break;
			case cmdPID_Roll_D_DEC:
                          DrYL_Roll_PID_Dec_D();
			break;
				
		}
		
	}
	
}

float DrYL_Abs(float d)
{
    if(d>=0)return d;
    else   return (-d);
}
void DrYL_Send_Sensor(float y)
{
    u8 sensor_uchar;
    u16 sensor_uint;
   // float y_positive;
    u8 three,two,one;
    bool sign_flag=0; //正负数标志位
    if(y>=0) sign_flag=1;// 正数为1
    else y=-y;
    if(sign_flag==1)AnBT_Uart3_Send_Char('+');//第一位  符号位
    else            AnBT_Uart3_Send_Char('-'); 
    
    if(y<0.01)
    {
        AnBT_Uart3_Send_Char(0x30+0);
        AnBT_Uart3_Send_Char('.');
        AnBT_Uart3_Send_Char(0x30+0);
        AnBT_Uart3_Send_Char(0x30+0);
    }
    else if(y<0.1)
    {
      one = (char)(y*100);
      AnBT_Uart3_Send_Char(0x30+0);
      AnBT_Uart3_Send_Char('.');
      AnBT_Uart3_Send_Char(0x30+0);
      AnBT_Uart3_Send_Char(0x30+one);
    }
    else if(y<1)
    {
        sensor_uchar=(u8)(y*100);
        two=(sensor_uchar/10);
        one=(sensor_uchar%10);
        AnBT_Uart3_Send_Char(0x30+0);
        AnBT_Uart3_Send_Char('.');
        AnBT_Uart3_Send_Char(0x30+two);
        AnBT_Uart3_Send_Char(0x30+one);
    }
    else if(y<10)
    {
        sensor_uint=(u16)(y*100);
        three=(u8)(sensor_uint/100);
        two=(u8)(sensor_uint%100/10);
        one=(u8)(sensor_uint%10);
        AnBT_Uart3_Send_Char(0x30+three);
        AnBT_Uart3_Send_Char('.');
        AnBT_Uart3_Send_Char(0x30+two);
        AnBT_Uart3_Send_Char(0x30+one);
    }
    else if(y<100)
    {
        sensor_uint=(u16)(y*10);
        three=(u8)(sensor_uint/100);
        two=(u8)(sensor_uint%100/10);
        one=(u8)(sensor_uint%10);
        AnBT_Uart3_Send_Char(0x30+three);
        AnBT_Uart3_Send_Char(0x30+two);
        AnBT_Uart3_Send_Char('.');
        AnBT_Uart3_Send_Char(0x30+one);
    }
    else
    {
        sensor_uchar=(u8)y;
        three=sensor_uchar/100;
        two=sensor_uchar%100/10;
        one=sensor_uchar%10;
        AnBT_Uart3_Send_Char(0x30+three);
        AnBT_Uart3_Send_Char(0x30+two);
        AnBT_Uart3_Send_Char(0x30+one);
        AnBT_Uart3_Send_Char('.');
    }
 
}
void DrYL_Send_Yaw(float y)
{
    AnBT_Uart3_Send_Char('Y');
    DrYL_Send_Sensor(y);
}
void DrYL_Send_Roll(float r)
{
   AnBT_Uart3_Send_Char('R');
   DrYL_Send_Sensor(r);
}
void DrYL_Send_Pitch(float p)
{
    AnBT_Uart3_Send_Char('P');
    DrYL_Send_Sensor(p);
}

void DrYL_Send_Moto_Data(u16 d)
{
     u8 four,three,two,one;
     four=(u8)(d/1000);
     three=(u8)(d%1000/100);
     two=(u8)(d%100/10);
     one=(u8)(d%10);
     if(four>0)AnBT_Uart3_Send_Char(0x30+four);
     else AnBT_Uart3_Send_Char(' ');
     AnBT_Uart3_Send_Char(0x30+three);
     AnBT_Uart3_Send_Char(0x30+two);
     AnBT_Uart3_Send_Char(0x30+one);  
}
///
void DrYL_Send_Moto_XP_Data(u16 d)
{
    AnBT_Uart3_Send_Char('A');
    DrYL_Send_Moto_Data(d);
}
void DrYL_Send_Moto_XN_Data(u16 d)
{
    AnBT_Uart3_Send_Char('B');
    DrYL_Send_Moto_Data(d);
}
void DrYL_Send_Moto_YP_Data(u16 d)
{
    AnBT_Uart3_Send_Char('C');
    DrYL_Send_Moto_Data(d);
}

void DrYL_Send_Moto_YN_Data(u16 d)
{
    AnBT_Uart3_Send_Char('D');
    DrYL_Send_Moto_Data(d);
}
//发送上下左右的电机转数
//xp  X_Positive
//xn  X_Negative
//yp  Y_Positive
//yn  Y_Negative
void DrYL_Send_Moto(u16 xp ,u16 xn, u16 yp, u16 yn)
{
      DrYL_Send_Moto_XP_Data(xp);
      DrYL_Send_Moto_XN_Data(xn);
      DrYL_Send_Moto_YP_Data(yp);
      DrYL_Send_Moto_YN_Data(yn);
}
/*
void DrYL_Send_PID_INC_P(float d)
{
	AnBT_Uart3_Send_Char('D');
}*/
/****************************************************
//
//
**************************************************/
void DrYL_Pitch_PID_Inc_P(void)
{
   pitch_pid_new.Proportion+=CHANGE_NUMBER;
}
void DrYL_Pitch_PID_Inc_I(void)
{
   pitch_pid_new.Integral+=CHANGE_NUMBER;
}
void DrYL_Pitch_PID_Inc_D(void)
{
   pitch_pid_new.Derivative+=CHANGE_NUMBER;
}
/************************************************/
void DrYL_Pitch_PID_Dec_P(void)
{
   pitch_pid_new.Proportion-=CHANGE_NUMBER;
}
void DrYL_Pitch_PID_Dec_I(void)
{
   pitch_pid_new.Integral-=CHANGE_NUMBER;
}
void DrYL_Pitch_PID_Dec_D(void)
{
   pitch_pid_new.Derivative-=CHANGE_NUMBER;
}
/**************************************************
**************************************************/

void DrYL_Roll_PID_Inc_P(void)
{
   roll_pid_new.Proportion+=CHANGE_NUMBER;
}
void DrYL_Roll_PID_Inc_I(void)
{
   roll_pid_new.Integral+=CHANGE_NUMBER;
}
void DrYL_Roll_PID_Inc_D(void)
{
   roll_pid_new.Derivative+=CHANGE_NUMBER;
}
/*************************************************************/
void DrYL_Roll_PID_Dec_P(void)
{
   roll_pid_new.Proportion-=CHANGE_NUMBER;
}
void DrYL_Roll_PID_Dec_I(void)
{
   roll_pid_new.Integral-=CHANGE_NUMBER;
}
void DrYL_Roll_PID_Dec_D(void)
{
   roll_pid_new.Derivative-=CHANGE_NUMBER;
}
/*************************************************
**************************************************/
/*void DrYL_Send_PID_INC_I(float d)
{

}
void DrYL_Send_Pitch_PID_INC_D(float d)
{
    
}
void DrYL_Send_Pitch_PID_DEC_P(float d)
{
}
void DrYL_Send_Pitch_PID_DEC_I(float d)
{
}
void DrYL_Send_PID_DEC_D(float d)
{
}*/
/*
void DrYL_Send_PID_Data(float d)
{
    bool sign_flag=0; //正负数标志位
    if(d>=0) sign_flag=1;// 正数为1
    else d=-d;
    if(sign_flag==1)AnBT_Uart3_Send_Char('+');//第一位  符号位
    else            AnBT_Uart3_Send_Char('-'); 
    
    
}*/
char frame=0;

/****************************************************
发送一帧数据，包括航向角、俯仰角、翻滚角
*******************************************************/
void DrYLSendOneFrameData(void)
{
  AnBT_Uart3_Send_Char('S');
  switch(frame)
  {
  case 0:
    AnBT_Uart3_Send_Char('1');
    DrYL_Send_Sensor(Yaw); //发送航向角的信息
    break;
  case 1:
    AnBT_Uart3_Send_Char('2');
    DrYL_Send_Sensor(Roll); //发送横滚角
    break;
  case 2:
    AnBT_Uart3_Send_Char('3');
    DrYL_Send_Sensor(Pitch);// 发送俯仰角
    break;
  case 3:
    AnBT_Uart3_Send_Char('4');
    DrYL_Send_Moto_Data(Moto_X_Positive);
    DrYL_Send_Moto_Data(' ');
    break;
  case 4:
    AnBT_Uart3_Send_Char('5');
    DrYL_Send_Moto_Data(Moto_X_Negative);
    DrYL_Send_Moto_Data(' ');
    break;
  case 5:
    AnBT_Uart3_Send_Char('6');
    DrYL_Send_Moto_Data(Moto_Y_Positive);
    DrYL_Send_Moto_Data(' ');
    break;
  case 6:
    AnBT_Uart3_Send_Char('7');
    DrYL_Send_Moto_Data(Moto_Y_Negative);
    DrYL_Send_Moto_Data(' ');
    break;
  case 7:
    AnBT_Uart3_Send_Char('8');
    DrYL_Send_Sensor(pitch_pid_new.Proportion);
    break;
  case 8:
    AnBT_Uart3_Send_Char('9');
    DrYL_Send_Sensor(pitch_pid_new.Integral);
    break;
  case 9:
    AnBT_Uart3_Send_Char('a');
    DrYL_Send_Sensor(pitch_pid_new.Derivative);
    break;
  case 10:
    AnBT_Uart3_Send_Char('b');
    DrYL_Send_Sensor(roll_pid_new.Proportion);
    break;
  case 11:
    AnBT_Uart3_Send_Char('c');
    DrYL_Send_Sensor(roll_pid_new.Integral);
    break;
  case 12:
    AnBT_Uart3_Send_Char('d');
    DrYL_Send_Sensor(roll_pid_new.Derivative);
    break;
  default:
    break;
  }
  frame++;
  if(frame==13)frame=0;
  AnBT_Uart3_Send_Char('E');
 
}