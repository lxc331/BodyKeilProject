#ifndef FXAS2100_H
#define FXAS2100_H


#include "common.h"
#include "SoftIIC.h"


#define IIC_FXAS2100_ADR          0x20
#define FXAS2100_CTRL_REG0        0x0D          ////        0x01   1000dps
#define FXAS2100_CTRL_REG1        0x13          ///         0x00  Stand By    0x02  Active  800HZ
#define FXAS2100_CTRL_REG2        0x14          ////        0x03
#define FXAS2100_ID               0x0C          /////       ID  0xD7
#define FXAS2100_XDATA            0x01


typedef struct{
  float GYROXdata;
  float GYROYdata;
  float GYROZdata;
}FXAS2100Datatypedef;


bool FXAS2100_init(void);
bool FXAS2100_DataRead(FXAS2100Datatypedef *Q);
#endif