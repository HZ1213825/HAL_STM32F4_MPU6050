@ [toc]
# 概述



> MPU6050是一个3轴陀螺仪(测角加速度)和3轴加速度计(测量线加速度)的测量芯片
>
> 内部自带运算单元(DMP),可以输出经姿态融合计算后的**四元数**(一种表示旋转的方法)

> 而且MPU6050的价格较低(10r以下),常被用于精度不高的场合作为姿态感知的芯片
>
> 如经典项目平衡车,某年电赛题目风力摆等



> MPU6050可以获取的数据为3轴的角加速度和三轴加速度,为了得到平常使用的欧拉角或者四元数,需要根据这些数据进行姿态解算
>
> 可以在单片机内部进行姿态解算,如使用 卡尔曼滤波 但是这样会占用大量单片机资源,因此常用MPU6050自带的运算单元来进行解算



> 注意,本文的代码借是将 正点原子,大鱼电子,DMP的官方库 进行了整合修改,并非100%原创
>
> 注意,本文的代码借是将 正点原子,大鱼电子,DMP的官方库 进行了整合修改,并非100%原创
>
> 注意,本文的代码借是将 正点原子,大鱼电子,DMP的官方库 进行了整合修改,并非100%原创

# 硬件设计

> 使用官方数据手册给出的电路图进行设计,实测可以使用

![请添加图片描述](https://img-blog.csdnimg.cn/04770ea2dd1e42aabce5c1dada4b2495.png)


> 要注意CPOUT(20脚)的电容,这是个电荷泵电容,一定要使用2.2nF,不然会出现不稳定

![](https://img-blog.csdnimg.cn/35f6c6d3c5a240abb477cfe453c148d2.png)


> MPU6050使用标准I2C通信,建议在I2C加外部上拉电阻(也可以将单片机内设为开漏上拉输出)
>
> 6,7号引脚连接的是地磁计的用于修正偏航角(Yaw),同样是I2C通信

> 封装是比较难焊的QFN-24,建议使用风枪或者加热台焊接(用烙铁太折磨人了)

![](https://img-blog.csdnimg.cn/21269a6a85b349858c1ea54e0280e95c.png)


> 这是对应的角加速度和加速度的方向,以及解算后的欧拉角
>
> 另外因为硬件原因,偏航角(Yaw)会出现偏移,这是无可避免的,可以通过外接地磁计改善

# 软件设计

## I2C通信

> MPU6050的I2C从器件地址是依据AD0(9号引脚)的电平而变化的
>
> AD0引脚为低电平(通过10K电阻接地)则地址是 0x68 << 1 = 0xD0 (也就是常说的除最低位外的地址)
>
> AD0引脚为高电平则地址是 0x69 << 1 = 0xD2 (也就是常说的除最低位外的地址)
>
> 最后一位和标准I2C一样是根据读写来确定的
>
> **地址要特别注意,容易出错**



I2C协议没什么好说的,请看这篇博客

[传送门](https://blog.csdn.net/m0_57585228/article/details/124700091)

> 通过宏定义来选择硬件和软件I2C

```c
/*
硬件I2C模式
需要：
1.I2C
    I2C
    (默认设置)
    标准模式
    时钟频率100kHz
    地址长度7bit
    不用填写设备地址
取消下方注释
*/

// extern I2C_HandleTypeDef hi2c2;
// #define MPU6050_I2C_Handle hi2c2
// #define MPU6050_Hardware_I2C

/*
软件I2C模式
需要：
1.GPIO 2个
    均为开漏输出（上不上拉取决于外部电路）
    最高等级
取消下方注释,按照自己的管脚更改即可
*/

#define MPU6050_Software_I2C

#ifdef MPU6050_Software_I2C
#define I2C_Group_SCL GPIOA // I2C的时钟GPIO组号
#define I2C_SCL GPIO_PIN_0  // I2C时钟的GPIO端口号

#define I2C_Group_SDA GPIOA // I2C的数据GPIO组号
#define I2C_SDA GPIO_PIN_1  // I2C数据的GPIO端口号

#define I2C_Write_SCL(x) HAL_GPIO_WritePin(I2C_Group_SCL, I2C_SCL, x)
#define I2C_Write_SDA(x) HAL_GPIO_WritePin(I2C_Group_SDA, I2C_SDA, x)

#define I2C_Read_SCL() HAL_GPIO_ReadPin(I2C_Group_SCL, I2C_SCL)
#define I2C_Read_SDA() HAL_GPIO_ReadPin(I2C_Group_SDA, I2C_SDA)
#endif
```

```c
#include "mpuiic.h"
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

```

> **特别注意:使用软件I2C时请注意延迟函数,不要过快(通信失败),不要过慢(通信时间长)**`void I2C_Delay()`

## MPU6050设置

> 这部分是借鉴自正点原子的内容

```c
#include "mpu6050.h"
#include "delay.h"
#include "usart.h"
#include "i2c.h"
#include "Print.h"
/**
 * @brief 初始化MPU6050
 * @param 无
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 14:51:59
 */
uint8_t MPU_Init(void)
{
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6050
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					 //陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					 //加速度传感器,±2g
	MPU_Set_Rate(50);						 //设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	 //关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 //关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	MPU_Read_Byte(MPU_DEVICE_ID_REG);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为参考
	MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
	MPU_Set_Rate(200);						 //设置采样率为50Hz
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //设置数字低通滤波器
	return MPU_Set_LPF(rate / 2);					  //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
	;
}
//得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
}
//得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	return res;
	;
}
```

> 比较常用的是
>
> `short MPU_Get_Temperature(void)`  获取温度
>
> `uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)`获取角加速度
>
> `uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)`获取线加速度



> 这是MPU6050的初始化函数,将器件ID的校验去掉了(因为我这边有个品牌的芯片读取ID不正确但可以正常使用)
>
> 在程序开始时调用即可
>
> `uint8_t MPU_Init(void)`

## DMP设置

> 这部分是来自官方库
>
> DMP部分是不开源的,不然也不需要使用官方库了

> 需要更改`inv_mpu.c`里的几个API,关于时间,I2C和打印信息的部分

![](https://img-blog.csdnimg.cn/4bd4e05d713e4244aff0753ccf901e17.png)


> 需要将`int mpu_init(void)`函数(在`inv_mpu.c`)中的这段注释并改成这个(光标选中的)

![](https://img-blog.csdnimg.cn/c0997f31eefb47e0bb391d88e825f8bc.png)


> 重写读取欧拉角和初始化函数,主要是去掉了几个验证

```c
/**
 * @brief 初始化DMP
 * @param 无
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 14:50:05
 */
uint8_t mpu_dmp_init(void)
{
    uint8_t res = 0;
    if (mpu_init() == 0) //初始化MPU6050
    {
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置所需要的传感器
        if (res)
            return 1;
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置FIFO
        if (res)
            return 2;
        res = mpu_set_sample_rate(DEFAULT_MPU_HZ); //设置采样率
        if (res)
            return 3;
        res = dmp_load_motion_driver_firmware(); //加载dmp固件
        if (res)
            return 4;
        res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)); //设置陀螺仪方向
        if (res)
            return 5;
        res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | //设置dmp功能
                                 DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                 DMP_FEATURE_GYRO_CAL);
        if (res)
            return 6;
        res = dmp_set_fifo_rate(DEFAULT_MPU_HZ); //设置DMP输出速率(最大不超过200Hz)
        if (res)
            return 7;
        res = run_self_test();      //自检
                                    //	if(res)return 8;
        res = mpu_set_dmp_state(1); //使能DMP
        if (res)
            return 9;
    }
    else
        return 10;
    return 0;
}
/**
 * @brief 得到DMP处理后的欧拉角
 * @param pitch:俯仰角( -90° - 90° )
 * @param roll:横滚角( -180° - 180° )
 * @param yaw:航向角( -180° - 180° )
 * @return 状态 0成功 其他失败
 * @author HZ12138
 * @date 2022-08-08 14:46:44
 */
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
     **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
     **/
    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / q30; // q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        //计算得到俯仰角/横滚角/航向角
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                    // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;     // roll
        *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }
    else
        return 2;
    return 0;
}

```

> 其他部分来自大鱼电子的移植库,无其他修改了

> **常用的函数就2个**

> **DMP初始化函数**
>
> **`uint8_t mpu_dmp_init(void)`**

> **获取欧拉角函数**
>
> **`uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw)`**

# 注意

> 1 通过DMP解算出的欧拉角顺序是 Z-Y-X (这个很重要)
>
> 2.因为没有地磁计,所以z(yow)的零点漂移会很严重,基本上没法看
>
> 3.mpu6050是相对位置,是基于上电时的位置计算的(但是好像DMP有自己的修正算法,可以让数据几乎保持绝对)
>
> 4.尽量在水平位置上电,即使dmp有修正也会出现10°以上的误差

# 成品

[GitHub](https://github.com/HZ1213825/HAL_STM32F4_MPU6050)

[百度网盘](https://pan.baidu.com/s/1mWYkC7z9uvKUbY5EhrEv_w?pwd=s7yx)

