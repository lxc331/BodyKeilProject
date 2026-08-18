#ifndef BMX055_H
#define BMX055_H

#include    <stdio.h>                       //printf
#include    <string.h>                      //memcpy
#include    <stdlib.h>                      //malloc
#include    "stdbool.h"
typedef unsigned char              uint8; 
typedef unsigned short int         uint16;
typedef unsigned long int          uint32;
typedef unsigned long long int     uint64;

typedef long long int               int64;
typedef char                        int8;
typedef short int                   int16;
typedef long  int                   int32;

typedef volatile int8               vint8;   //  8 bits 
typedef volatile int16              vint16;  // 16 bits 
typedef volatile int32              vint32;  // 32 bits 

typedef volatile uint8              vuint8;  //  8 bits 
typedef volatile uint16             vuint16; // 16 bits 
typedef volatile uint32             vuint32; // 32 bits 

typedef volatile float             vfloat;

typedef struct{
  uint8 addr;
  uint8 val;
}Registermaptypedef;         //Íâ²¿¼Ä´æÆ÷
#define IIC_BMX055_ACC_ADR    0x18
#define IIC_BMX055_GYRO_ADR   0x68
#define IIC_BMX055_MAG_ADR    0x10

#define BMX055_ACC_XDATALSB   0x02
#define BMX055_ACC_ID         0x00
#define BMX055_ACC_PMURANGE   0x0F
#define BMX055_ACC_PMUBW      0x10
#define BMX055_ACC_PMULPM     0x11


#define BMX055_GYRO_XDATALSB  0x02
#define BMX055_GYRO_ID        0x00
#define BMX055_GYRO_RANGE     0x0F
#define BMX055_GYRO_BW        0x10
#define BMX055_GYRO_LPM       0x11
#define BMX055_GYRO_RATEHBW   0x13

#define BMX055_MAG_XDATALSB   0x42
#define BMX055_MAG_ID         0x40
#define BMX055_MAG_POM        0x4B
#define BMX055_MAG_DATARATE   0x4C
#define BMX055_MAG_INTEN      0x4E

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


bool BMX055_init(void);
bool BMX055_DataRead(BMX055Datatypedef *Q, uint8 type);
#endif