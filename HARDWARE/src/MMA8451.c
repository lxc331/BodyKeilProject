/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * All rights reserved.
 *中南大学黄竞辉
 *
 * @file       		MMA8451
 * @author     		Alex
 * @version    		v1.0
 * @Software 		IAR 8.1
 * @date       		2017-11-9
 ********************************************************************************************************************/

#include "MMA8451.h"


bool MMA8451_init(void)
{
  uint8 ErrCount = 0;
  while(IIC_Read_Reg(IIC_MMA8451_ADR, MMA8451_ID) != 0x1A)   /////确认芯片ID
  {
    ErrCount++;
    if(ErrCount > 5)
      return false;
  }
  if(IIC_Write_Reg(IIC_MMA8451_ADR, MMA8451_CTRL_REG1_REG, 0x00) == false)return false;  ///stand by
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MMA8451_ADR, MMA8451_XYZ_DATA_CFG_REG, 0x02) == false)return false;   ///1000dps
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MMA8451_ADR, MMA8451_CTRL_REG2_REG, 0x02) == false)return false;  
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MMA8451_ADR, MMA8451_CTRL_REG4_REG, 0x00) == false)return false;  
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MMA8451_ADR, MMA8451_CTRL_REG1_REG, 0x01) == false)return false;   ////800hz
  Common_delay(10);
  return true;
}

bool MMA8451_DataRead(MMA8451Datatypedef *Q)
{
  uint8 datatemp[6] = {0};
  if(IIC_Read_Buff(IIC_MMA8451_ADR, MMA8451_XDATA, datatemp, 6) == false)return false;
  Q->ACCXdata = (float)((int16)((datatemp[0] << 8) | datatemp[1]) >> 2);
  Q->ACCYdata = (float)((int16)((datatemp[2] << 8) | datatemp[3]) >> 2);
  Q->ACCZdata = (float)((int16)((datatemp[4] << 8) | datatemp[5]) >> 2);
  return true;
}