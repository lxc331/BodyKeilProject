/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * All rights reserved.
 *中南大学黄竞辉
 *
 * @file       		MPU3050
 * @author     		Alex
 * @version    		v1.0
 * @Software 		IAR 8.1
 * @date       		2017-11-9
 ********************************************************************************************************************/

#include "MPU3050.h"


bool MPU3050_init(void)
{
  uint8 ErrCount = 0;
  IIC_Write_Reg(IIC_MPU3050_ADR, MPU3050_PWR_MGM, 0x80);     /////3050和6050的通病  开头复位
  Common_delay(20);
  while(IIC_Read_Reg(IIC_MPU3050_ADR, MPU3050_ID) != 0x69)   /////确认芯片ID  有的是0x68,自己改吧
  {
    ErrCount++;
    if(ErrCount > 5)
      return false;
  }
  if(IIC_Write_Reg(IIC_MPU3050_ADR, MPU3050_SMPLRT_DIV, 0x00) == false)return false;   
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MPU3050_ADR, MPU3050_DLPF_FS_SYNC, 0x11) == false)return false;  
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MPU3050_ADR, MPU3050_INT_CFG, 0x00) == false)return false;  
  Common_delay(10);
  if(IIC_Write_Reg(IIC_MPU3050_ADR, MPU3050_PWR_MGM, 0x00) == false)return false;   
  Common_delay(10);
  return true;
}

bool MPU3050_DataRead(MPU3050Datatypedef *Q)
{
  uint8 datatemp[6] = {0};
  if(IIC_Read_Buff(IIC_MPU3050_ADR, MPU3050_XDATA, datatemp, 6) == false)return false;
  Q->GYROXdata = (float)((int16)((datatemp[0] << 8) | datatemp[1]));
  Q->GYROYdata = (float)((int16)((datatemp[2] << 8) | datatemp[3]));
  Q->GYROZdata = (float)((int16)((datatemp[4] << 8) | datatemp[5]));
  return true;
}