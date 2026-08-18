/* Includes ------------------------------------------------------------------*/

#ifndef __IMU_H
#define __IMU_H

#include <stm32f10x.h>
#include "sys.h"
#include <math.h>

#define AccSen							0.0097	//g/lsb @ +/- 2g
#define GyroSen							0.0305			//°/s/lsb @ 1000
#define TempSen							0.5			//K/LSB center temperature is 23℃
#define MagxySen						0.317	//uT/lsb
#define MagzSen							0.153	//uT/lsb

//SDO1 SDO2 CSB3 pulled to GND  三个传感器IIC的地址
#define Acc_addr						0x18
#define Gyro_addr						0x68
#define Mag_addr						0x10


/* BMX055 Register Map */
//ACC define
#define	ACC_ID							0x00	//OXFA
#define	ACC_XL							0x02
#define	ACC_XM							0x03
#define	ACC_YL							0x04
#define	ACC_YM							0x05
#define	ACC_ZL							0x06
#define	ACC_ZM							0x07
#define	Temp							0x08
#define ACC_range						0x0f	//1100b --> +/- 16g
#define Shasow_dis						0x13
#define ACC_ret							0x14	//write 0xb6
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F
//Gyro define
#define	GYRO_ID							0x00	//OXOF
#define	GYRO_XL							0x02
#define	GYRO_XM							0x03
#define	GYRO_YL							0x04
#define	GYRO_YM							0x05
#define	GYRO_ZL							0x06
#define	GYRO_ZM							0x07
#define GYRO_range						0x0f	//010b --> +/- 500°/s
#define GYRO_ret						0x14	//write 0xb6
//MAG define
#define	MAG_ID							0x40	//OX32
#define	MAG_XL							0x42
#define	MAG_XM							0x43
#define	MAG_YL							0x44
#define	MAG_YM							0x45
#define	MAG_ZL							0x46
#define	MAG_ZM							0x47
#define	MAG_RHAL						0x48
#define	MAG_RHAM						0x49
#define MAG_ret							0x4b	//1000 0001b


typedef struct{
  float Xdata;
  float Ydata;
  float Zdata;
}AttitudeDatatypedef;
typedef struct
{
  float GYROXdata;
  float GYROYdata;
  float GYROZdata;
  float ACCXdata;
  float ACCYdata;
  float ACCZdata;
  float MAGXdata;
  float MAGYdata;
  float MAGZdata;
}BMX055Datatypedef;
extern int IsGyroOffsetReset;  
#define M_PI  (float)3.1415926535
extern	float Acc_dat[3];
extern float Gyro_dat[3];
extern float Mag_dat[3];
extern u8	BMX055_MBUF[8];
extern AttitudeDatatypedef    GyroOffset,AgOffset;

extern AttitudeDatatypedef    MagOffset,MagOffset_J;
extern void get_acc(void);
extern void get_gyro(void);
extern float q[4];
/* Exported functions ------------------------------------------------------- */
void GyroOffset_init(void);
void IMU_Init(void);
void BMX055_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay,
                            int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void IMU_init_1(void);
uint32_t micros(void);
void IMU_getValues(float * values);
void IMU_getYawPitchRoll(float * angles);
void get_dat(void);
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif
