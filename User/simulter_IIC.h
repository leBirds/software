#ifndef __MY_SIMULTER_IIC_H__
#define __MY_SIMULTER_IIC_H__

extern void i2cInit(void);
extern void I2C_SendByte(uint8_t byte);
extern uint8_t I2C_ReceiveByte(void);
extern bool I2C_Start(void);
extern void I2C_Stop(void);
extern void I2C_Ack(void);
extern void I2C_NoAck(void);
extern bool I2C_WaitAck(void);
extern bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif