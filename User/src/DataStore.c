#include "include.h"
#include "Attitude_Calculation.h"
#include "DataStore.h"
#include "isr.h"

FLASH_WRITE_TYPE   FLASH_DATA[FLASHNUM];


bool My_FlashWrite(uint16 Boot)        /////写入
{
    FLASH_ERASE(BOOTSECTOR - Boot);
    for(int n = 0; n < FLASHNUM; n++)
    {
       FLASH_WRITE((BOOTSECTOR - Boot), (FLASH_ALIGN_ADDR * n), FLASH_DATA[n]);
       FLASH_DELAY(4000);
    }
    return true;
}

bool My_FlashRead(uint16 Boot)          ////读出
{
    for(int n = 0; n < FLASHNUM; n++)
    {
       FLASH_DATA[n] = FLASH_READ((BOOTSECTOR - Boot), (FLASH_ALIGN_ADDR * n), FLASH_WRITE_TYPE);
       FLASH_DELAY(4000);
    }
    return true;
}


extern AttitudeDatatypedef    GyroOffset;
void DataSave(void)                       ///////数据保存
{
  FLASH_DATA[0] = Float2U32(GyroOffset.Xdata);
  FLASH_DATA[1] = Float2U32(GyroOffset.Ydata);
  FLASH_DATA[2] = Float2U32(GyroOffset.Zdata);
  My_FlashWrite(0);
  return;
}

void DataLoad(void)                       ///////数据载入
{
  My_FlashRead(0);
  GyroOffset.Xdata = U322Float(&FLASH_DATA[0]);
  GyroOffset.Ydata = U322Float(&FLASH_DATA[1]);
  GyroOffset.Zdata = U322Float(&FLASH_DATA[2]);
}


void Data_init(void)
{
  FLASH_init();
  Common_delay(100);
  DataLoad();
}