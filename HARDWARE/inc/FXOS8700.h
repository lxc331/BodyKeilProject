#ifndef FXOS8700_H
#define FXOS8700_H


#include "common.h"
#include "SOFTIIC.h"


#define IIC_FXOS8700_ADR          0X1E
#define FXOS8700_ID               0x0D          /////ID  0xD7
#define FXOS8700_XYZ_DATA_CFG     0x0E          ////0x01  4G
#define FXOS8700_SYSMOD           0x0B
#define FX0S8700_CTRL_REG1        0x2A 
#define FXOS8700_ACC_XDATA        0x01
#define FXOS8700_MAG_XDATA        0x33

typedef struct{
  float ACCXdata;
  float ACCYdata;
  float ACCZdata;
  float MAGXdata;
  float MAGYdata;
  float MAGZdata;
}FXOS8700Datatypedef;

bool FXOS8700_DataRead(FXOS8700Datatypedef *Q, uint8 type);
bool FXOS8700_init(void);
#endif

