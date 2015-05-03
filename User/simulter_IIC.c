#include "includes.h"
/****************
*****Pin not defintion
***********/

#define SCL_H         GPIO_SetBits(GPIOA, GPIO_Pin_4)// GPIOA->BSRR = GPIO_Pin_2 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIO_ResetBits(GPIOA, GPIO_Pin_4)//GPIOA->BRR  = GPIO_Pin_2 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIO_SetBits(GPIOA, GPIO_Pin_5)//GPIOA->BSRR = GPIO_Pin_3 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIO_ResetBits(GPIOA, GPIO_Pin_5)//GPIOA->BRR  = GPIO_Pin_3 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOA->IDR  & GPIO_Pin_4 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOA->IDR  & GPIO_Pin_5 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */


void i2cInit(void)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
}
void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
         i--;
}

bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    return true;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}
 void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) 
    {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) 
    {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}


bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}



uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) 
    {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_Ack();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}




