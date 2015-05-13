#include "includes.h"

#define STA_START        0        //��ͷ����״̬��־ 
#define STA_COMMD        1        //�������״̬��־
#define STA_END          2        //��β����״̬��־
u8  rx_buffer[50];           // ����
static u8  START_STRING[5] = "Head";      //���ݰ�ͷ      
static u8  END_STRING[5]   = "Tail";      //���ݰ�β
static u8  *pstart    = START_STRING;   //ָ������ͷ��ָ��
static u8  *pend      = END_STRING;     //ָ������β��ָ��
static u8  *Prx       = rx_buffer;   //ָ����յ������ݵ�ָ��
u8    index1       = 0;           //����
u8    index2       = 0;           //����

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
	   if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)	//Բ�㲩ʿ:�������Ǵ��ڽ����ж�
	   {
	        u8 temp;
	        static u8 state = STA_START ; 
	        temp=USART_ReceiveData(USART3);
	        USART_ClearFlag(USART3,USART_FLAG_RXNE);	//Բ�㲩ʿ:����жϱ�־
	        
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
				                     index2 = 0;  // ������λ
				                     
				                     pend    	= END_STRING;  //  ָ���λ    
				                     pstart  	= START_STRING; 
				                     Prx       	= rx_buffer;      
				                     state    	= STA_START;     
						      com_read_flag =1; //����һ֡������ɣ���־λ��һ
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
		com_read_flag =0;// ��־λ����
		cmd = rx_buffer[0];
		switch(cmd)
		{
			case cmdStart :
				DrYL_Moto_Start();
                               // AnBT_Uart3_Send_Char('1');  // ����
                               // AnBT_Uart3_Send_Char('2');   // ����
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
		}
		
	}
	
}

float DrYL_Abs(float d)
{
    if(d>0)return d;
    else   return (-d);
}
void DrYL_Send_Sensor(float y)
{
    u8 sensor_uchar;
    u16 sensor_uint;
   // float y_positive;
    u8 three,two,one;
    bool sign_flag=0; //��������־λ
    if(y>=0) sign_flag=1;// ����Ϊ1
    else y=-y;
    if(sign_flag==1)AnBT_Uart3_Send_Char('+');//��һλ  ����λ
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
//�����������ҵĵ��ת��
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