/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * All rights reserved.
 *中南大学黄竞辉
 *
 * @file       		FXOS8700
 * @author     		Alex
 * @version    		v1.0
 * @Software 		IAR 8.1
 * @date       		2017-11-9
 ********************************************************************************************************************/

#include "FXOS8700.h"


bool FXOS8700_init(void)
{
  uint8 ErrCount = 0;
  while(IIC_Read_Reg(IIC_FXOS8700_ADR, FXOS8700_ID) != 0xC7)   /////确认芯片ID
  {
    ErrCount++;
    if(ErrCount > 5)
      return false;
  }
  if(IIC_Write_Reg(IIC_FXOS8700_ADR, FXOS8700_SYSMOD, 0x00) == false)return false;          ///standby
  Common_delay(10);
  if(IIC_Write_Reg(IIC_FXOS8700_ADR, FXOS8700_XYZ_DATA_CFG, 0x01) == false)return false;    ///4G
  Common_delay(10);
  if(IIC_Write_Reg(IIC_FXOS8700_ADR, FX0S8700_CTRL_REG1, 0x05) == false)return false;       ///
  Common_delay(10); 
  return true;
}

bool FXOS8700_DataRead(FXOS8700Datatypedef *Q, uint8 type)
{
  uint8 datatemp[6] = {0};
  if(IIC_Read_Buff(IIC_FXOS8700_ADR, FXOS8700_ACC_XDATA, datatemp, 6) == false)return false;
  Q->ACCXdata = (float)((int16)((datatemp[0] << 8) | datatemp[1]) >> 2);
  Q->ACCYdata = (float)((int16)((datatemp[2] << 8) | datatemp[3]) >> 2);
  Q->ACCZdata = (float)((int16)((datatemp[4] << 8) | datatemp[5]) >> 2);
  if(type)
  {
    if(IIC_Read_Buff(IIC_FXOS8700_ADR, FXOS8700_MAG_XDATA, datatemp, 6) == false)return false;
  }
  return true;
}