#ifndef MMA8451_H
#define MMA8451_H


#include "common.h"
#include "SOFTIIC.h"


#define IIC_MMA8451_ADR           0X1C
#define MMA8451_ID                0X0D
#define MMA8451_XYZ_DATA_CFG_REG  0x0E        /////Êä³öÄ£Ê½
#define MMA8451_CTRL_REG1_REG     0x2A
#define MMA8451_CTRL_REG2_REG     0x2B
#define MMA8451_CTRL_REG3_REG     0x2C
#define MMA8451_CTRL_REG4_REG     0x2D
#define MMA8451_CTRL_REG5_REG     0x2E
#define MMA8451_XDATA             0x01



typedef struct{
  float ACCXdata;
  float ACCYdata;
  float ACCZdata;
}MMA8451Datatypedef;

bool MMA8451_init(void);
bool MMA8451_DataRead(MMA8451Datatypedef *Q);
#endif