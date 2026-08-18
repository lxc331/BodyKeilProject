#include "LSM9DS0.h"

bool LSM9DS0_init(void)
{
  uint8 ErrCount = 0;
  while(IIC_Read_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_GYRO_ID) != 0xD4)   /////确认芯片ID
  {
    ErrCount++;
    if(ErrCount > 5)
      return false;
  }
  if(IIC_Write_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_CTRL_REG1_G, 0xCF) == false)return false;          ///standby
  Common_delay(10);
  if(IIC_Write_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_CTRL_REG2_G, 0x00) == false)return false;    ///4G
  Common_delay(10);
  if(IIC_Write_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_CTRL_REG3_G, 0x00) == false)return false;       ///
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_CTRL_REG4_G, 0x20) == false)return false;       ///2000DPS
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_CTRL_REG5_G, 0x00) == false)return false;       ///
  Common_delay(10); 
  
  
  ErrCount = 0;
  while(IIC_Read_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_ACCMAG_ID) != 0x49)   /////确认芯片ID
  {
    ErrCount++;
    if(ErrCount > 5)
      return false;
  }
  
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_INT_CTRL_REG_M, 0x00) == false)return false;          ///standby
  Common_delay(10);
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG0_XM, 0x00) == false)return false;    ///4G
  Common_delay(10);
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG1_XM, 0xA7) == false)return false;       ///
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG2_XM, 0x08) == false)return false;       ///4G
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG3_XM, 0x00) == false)return false;       ///
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG4_XM, 0x00) == false)return false;       ///
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG5_XM, 0x14) == false)return false;       ///
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG6_XM, 0x60) == false)return false;       ///8G
  Common_delay(10); 
  if(IIC_Write_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_CTRL_REG7_XM, 0x00) == false)return false;       ///
  Common_delay(10); 
  return true;
}

bool LSM9DS0_DataRead(LSM9DS0Datatypedef *Q, uint8 type)
{
  uint8 datatemp[6] = {0};
  for(uint8 i = 0; i < 6; i++)
    datatemp[i] = IIC_Read_Reg(IIC_LSM9DS0_GYROADR, LSM9DS0_XYZDATA_G + i);

  //if(IIC_Read_Buff(IIC_LSM9DS0_GYROADR, LSM9DS0_XYZDATA_G, datatemp, 2) == false)return false;
  Q->GYROXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]));
  Q->GYROYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]));
  Q->GYROZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]));
  //if(IIC_Read_Buff(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_XYZDATA_A, datatemp, 6) == false)return false;
  for(uint8 i = 0; i < 6; i++)
    datatemp[i] = IIC_Read_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_XYZDATA_A + i);
  Q->ACCXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]));
  Q->ACCYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]));
  Q->ACCZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]));
  /************************************/
  /*磁力计数据读取*/
  /************************************/
  if(type)
  {
    //if(IIC_Read_Buff(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_XYZDATA_M, datatemp, 6) == false)return false;
    for(uint8 i = 0; i < 6; i++)
      datatemp[i] = IIC_Read_Reg(IIC_LSM9DS0_ACCMAGADR, LSM9DS0_XYZDATA_M + i);
    Q->MAGXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]));
    Q->MAGYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]));
    Q->MAGZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]));
  }
  return true;
}

