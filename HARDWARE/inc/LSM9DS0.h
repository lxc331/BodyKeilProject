#ifndef LSM9DS0_H
#define LSM9DS0_H


#include "common.h"
#include "SOFTIIC.h"


#define IIC_LSM9DS0_GYROADR       0x6B
#define IIC_LSM9DS0_ACCMAGADR     0x1D

#define LSM9DS0_GYRO_ID           0x0F
#define LSM9DS0_CTRL_REG1_G       0x20      ////default  0x07  set 0x0f
#define LSM9DS0_CTRL_REG2_G       0x21      ////set 0x00
#define LSM9DS0_CTRL_REG3_G       0x22      ////set 0x00
#define LSM9DS0_CTRL_REG4_G       0x23      ////set 0x20      2000dps
#define LSM9DS0_CTRL_REG5_G       0x24      ////set 0x00
#define LSM9DS0_XYZDATA_G         0x28


#define LSM9DS0_ACCMAG_ID         0x0F
#define LSM9DS0_INT_CTRL_REG_M    0x12     ////set 0x00
#define LSM9DS0_CTRL_REG0_XM      0x1F     ////set 0x00
#define LSM9DS0_CTRL_REG1_XM      0x20     ////set 0xA7   1600HZ
#define LSM9DS0_CTRL_REG2_XM      0x21     ////set 0x18    8G
#define LSM9DS0_CTRL_REG3_XM      0x22     ////set 0x00
#define LSM9DS0_CTRL_REG4_XM      0x23     ////set 0x00
#define LSM9DS0_CTRL_REG5_XM      0x24     ////set 0x14
#define LSM9DS0_CTRL_REG6_XM      0x25     ////set 0x80    8G
#define LSM9DS0_CTRL_REG7_XM      0x26     ////set 0x00
#define LSM9DS0_XYZDATA_A         0x28
#define LSM9DS0_XYZDATA_M         0x08


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
}LSM9DS0Datatypedef;


bool LSM9DS0_DataRead(LSM9DS0Datatypedef *Q, uint8 type);
bool LSM9DS0_init(void);


#endif