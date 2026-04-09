#ifndef __MYI2C_H
#define __MYI2C_H

void MyI2C_Init(void);
void MyI2C_Start(uint8_t id);
void MyI2C_Stop(uint8_t id);
void MyI2C_SendByte(uint8_t id,uint8_t Byte);
uint8_t MyI2C_ReceiveByte(uint8_t id);
void MyI2C_SendAck(uint8_t id,uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(uint8_t id);
#endif
