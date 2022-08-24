#include "mpuiic.h"
#include "delay.h"
#if defined(MPU6050_Software_I2C)
/**
 * @brief 一段延迟
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-07-27 08:53:30
 */
void I2C_Delay()
{
	int z = 0xff;
	while (z--)
		;
}
/**
 * @brief 产生I2C起始信号
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-07-27 08:54:48
 */
void I2C_Start(void)
{
	I2C_Write_SDA(GPIO_PIN_SET);   //需在SCL之前设定
	I2C_Write_SCL(GPIO_PIN_SET);   // SCL->高
	I2C_Delay();				   //延时
	I2C_Write_SDA(GPIO_PIN_RESET); // SDA由1->0,产生开始信号
	I2C_Delay();				   //延时
	I2C_Write_SCL(GPIO_PIN_RESET); // SCL->低
}
/**
 * @brief 产生I2C结束信号
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-07-27 08:57:03
 */
void I2C_End(void)
{
	I2C_Write_SDA(GPIO_PIN_RESET); //在SCL之前拉低
	I2C_Write_SCL(GPIO_PIN_SET);   // SCL->高
	I2C_Delay();				   //延时
	I2C_Write_SDA(GPIO_PIN_SET);   // SDA由0->1,产生结束信号
	I2C_Delay();				   //延时
}
/**
 * @brief 发送应答码
 * @param ack:0 应答 1 不应达
 * @return 无
 * @author HZ12138
 * @date 2022-07-27 09:03:38
 */
void IIC_Send_ACK(uint8_t ack)
{
	if (ack == 1)
		I2C_Write_SDA(GPIO_PIN_SET); //产生应答电平
	else
		I2C_Write_SDA(GPIO_PIN_RESET);
	I2C_Delay();
	I2C_Write_SCL(GPIO_PIN_SET);   //发送应答信号
	I2C_Delay();				   //延时至少4us
	I2C_Write_SCL(GPIO_PIN_RESET); //整个期间保持应答信号
}
/**
 * @brief 接受应答码
 * @param 无
 * @return 应答码 0 应答 1 不应达
 * @author HZ12138
 * @date 2022-07-27 09:04:28
 */
uint8_t IIC_Get_ACK(void)
{
	uint8_t ret;				 //用来接收返回值
	I2C_Write_SDA(GPIO_PIN_SET); //电阻上拉,进入读
	I2C_Delay();
	I2C_Write_SCL(GPIO_PIN_SET); //进入应答检测
	I2C_Delay();				 //至少延时4us
	ret = I2C_Read_SDA();		 //保存应答信号
	I2C_Write_SCL(GPIO_PIN_RESET);
	return ret;
}
/**
 * @brief I2C写1Byte
 * @param dat:1Byte数据
 * @return 应答结果 0 应答 1 不应达
 * @author HZ12138
 * @date 2022-07-27 09:05:14
 */
uint8_t I2C_SendByte(uint8_t dat)
{
	uint8_t ack;
	for (int i = 0; i < 8; i++)
	{
		// 高在前低在后
		if (dat & 0x80)
			I2C_Write_SDA(GPIO_PIN_SET);
		else
			I2C_Write_SDA(GPIO_PIN_RESET);
		I2C_Delay();
		I2C_Write_SCL(GPIO_PIN_SET);
		I2C_Delay(); //延时至少4us
		I2C_Write_SCL(GPIO_PIN_RESET);
		dat <<= 1; //低位向高位移动
	}

	ack = IIC_Get_ACK();

	return ack;
}
/**
 * @brief I2C读取1Byte数据
 * @param ack:应答 0 应答 1 不应达
 * @return 接受到的数据
 * @author HZ12138
 * @date 2022-07-27 09:06:13
 */
uint8_t I2C_ReadByte(uint8_t ack)
{
	uint8_t ret = 0;
	// OLED_Read_SDA() 设置输入方向
	I2C_Write_SDA(GPIO_PIN_SET);
	for (int i = 0; i < 8; i++)
	{
		ret <<= 1;
		I2C_Write_SCL(GPIO_PIN_SET);
		I2C_Delay();
		// 高在前低在后
		if (I2C_Read_SDA())
		{
			ret++;
		}
		I2C_Write_SCL(GPIO_PIN_RESET);
		I2C_Delay();
	}

	IIC_Send_ACK(ack);

	return ret;
}
#endif
/**
 * @brief MUP6050 I2C连续写
 * @param addr:器件地址
 * @param reg:寄存器地址
 * @param len:长度
 * @param buf:缓冲区地址
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 15:47:11
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
#if defined(MPU6050_Software_I2C)
	uint8_t i;
	I2C_Start();
	I2C_SendByte((addr << 1) | 0); //发送器件地址+写命令
	I2C_SendByte(reg);			   //写寄存器地址
	for (i = 0; i < len; i++)
	{
		I2C_SendByte(buf[i]); //发送数据
	}
	I2C_End();
	return 0;
#elif defined(MPU6050_Hardware_I2C)
	HAL_I2C_Mem_Write(&MPU6050_I2C_Handle, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
	return 0;
#endif
}
/**
 * @brief MUP6050 I2C连续读
 * @param addr:器件地址
 * @param reg:寄存器地址
 * @param len:长度
 * @param buf:缓冲区地址
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 15:47:11
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
#if defined(MPU6050_Software_I2C)
	I2C_Start();
	I2C_SendByte((addr << 1) | 0); //发送器件地址+写命令
	I2C_SendByte(reg);			   //写寄存器地址
	I2C_Start();
	I2C_SendByte((addr << 1) | 1); //发送器件地址+读命令
	while (len)
	{
		if (len == 1)
			*buf = I2C_ReadByte(1); //读数据,发送nACK
		else
			*buf = I2C_ReadByte(0); //读数据,发送ACK
		len--;
		buf++;
	}
	I2C_End(); //产生一个停止条件
	return 0;
#elif defined(MPU6050_Hardware_I2C)
	HAL_I2C_Mem_Read(&MPU6050_I2C_Handle, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
	HAL_Delay(1);
	return 0;
#endif
}
/**
 * @brief MUP6050 I2C写一个字节
 * @param reg:寄存器地址
 * @param data:数据
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 15:47:11
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
#if defined(MPU6050_Software_I2C)
	I2C_Start();
	I2C_SendByte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
	I2C_SendByte(reg);				   //写寄存器地址
	I2C_SendByte(data);				   //发送数据
	I2C_End();
	return 0;
#elif defined(MPU6050_Hardware_I2C)
	return HAL_I2C_Mem_Write(&MPU6050_I2C_Handle, (MPU_ADDR << 1), reg, 1, &data, 1, 0xfff);
#endif
}
/**
 * @brief MUP6050 I2C写一个字节
 * @param reg:寄存器地址
 * @return 读取到的数据
 * @author HZ12138
 * @date 2022-08-08 15:47:11
 */
uint8_t MPU_Read_Byte(uint8_t reg)
{
#if defined(MPU6050_Software_I2C)
	uint8_t res;
	I2C_Start();
	I2C_SendByte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
	I2C_SendByte(reg);				   //写寄存器地址
	I2C_Start();
	I2C_SendByte((MPU_ADDR << 1) | 1); //发送器件地址+读命令
	res = I2C_ReadByte(1);			   //读取数据,发送nACK
	I2C_End();						   //产生一个停止条件
	return res;
#elif defined(MPU6050_Hardware_I2C)
	uint8_t zj;
	HAL_I2C_Mem_Read(&MPU6050_I2C_Handle, (MPU_ADDR << 1), reg, 1, &zj, 1, 0xfff);
	return zj;
#endif
}
