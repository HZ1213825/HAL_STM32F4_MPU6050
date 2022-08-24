#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"
#include "mpu6050.h"

// IIC所有操作函数
void I2C_Delay(void);              // MPU IIC延时函数
void I2C_Start(void);              //发送IIC开始信号
void I2C_End(void);                //发送IIC停止信号
uint8_t I2C_SendByte(uint8_t dat); // IIC发送一个字节
uint8_t I2C_ReadByte(uint8_t ack); // IIC读取一个字节
uint8_t MPU_Read_Byte(uint8_t reg);
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
#endif
