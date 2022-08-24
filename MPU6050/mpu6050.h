#ifndef __MPU6050_H
#define __MPU6050_H
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

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

//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR 0X68

////因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)
#define MPU_READ 0XD1
#define MPU_WRITE 0XD0

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG 0X0D   //自检寄存器X
#define MPU_SELF_TESTY_REG 0X0E   //自检寄存器Y
#define MPU_SELF_TESTZ_REG 0X0F   //自检寄存器Z
#define MPU_SELF_TESTA_REG 0X10   //自检寄存器A
#define MPU_SAMPLE_RATE_REG 0X19  //采样频率分频器
#define MPU_CFG_REG 0X1A          //配置寄存器
#define MPU_GYRO_CFG_REG 0X1B     //陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG 0X1C    //加速度计配置寄存器
#define MPU_MOTION_DET_REG 0X1F   //运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG 0X23      // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG 0X24  // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG 0X25 // IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG 0X26      // IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG 0X27 // IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG 0X28 // IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG 0X29      // IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG 0X2A // IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG 0X2B // IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG 0X2C      // IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG 0X2D // IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG 0X2E // IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG 0X2F      // IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG 0X30 // IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG 0X31 // IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG 0X32      // IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG 0X33   // IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG 0X34 // IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG 0X35   // IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG 0X36 // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG 0X37  //中断/旁路设置寄存器
#define MPU_INT_EN_REG 0X38     //中断使能寄存器
#define MPU_INT_STA_REG 0X3A    //中断状态寄存器

#define MPU_ACCEL_XOUTH_REG 0X3B //加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG 0X3C //加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG 0X3D //加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG 0X3E //加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG 0X3F //加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG 0X40 //加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG 0X41 //温度值高八位寄存器
#define MPU_TEMP_OUTL_REG 0X42 //温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG 0X43 //陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG 0X44 //陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG 0X45 //陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG 0X46 //陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG 0X47 //陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG 0X48 //陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG 0X63 // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG 0X64 // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG 0X65 // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG 0X66 // IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG 0X67 // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG 0X68  //信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG 0X69 //运动检测控制寄存器
#define MPU_USER_CTRL_REG 0X6A    //用户控制寄存器
#define MPU_PWR_MGMT1_REG 0X6B    //电源管理寄存器1
#define MPU_PWR_MGMT2_REG 0X6C    //电源管理寄存器2
#define MPU_FIFO_CNTH_REG 0X72    // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG 0X73    // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG 0X74      // FIFO读写寄存器
#define MPU_DEVICE_ID_REG 0X75    //器件ID寄存器

uint8_t
MPU_Init(void);                                                              //初始化MPU6050
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); // IIC连续写
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);  // IIC连续读
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);                           // IIC写一个字节
uint8_t MPU_Read_Byte(uint8_t reg);                                          // IIC读一个字节

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);

short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az);

#endif
