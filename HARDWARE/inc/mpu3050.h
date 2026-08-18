#ifndef MPU3050_H
#define MPU3050_H

#include "common.h"
#include "SOFTIIC.h"



#define IIC_MPU3050_ADR           0x68
#define MPU3050_ID                0x00
#define MPU3050_PWR_MGM           0x3E
#define MPU3050_SMPLRT_DIV        0x15
#define MPU3050_DLPF_FS_SYNC      0X16
#define MPU3050_INT_CFG           0X17
#define MPU3050_XDATA             0x1D


typedef struct{
  float GYROXdata;
  float GYROYdata;
  float GYROZdata;
}MPU3050Datatypedef;

bool MPU3050_init(void);
bool MPU3050_DataRead(MPU3050Datatypedef *Q);
#endif