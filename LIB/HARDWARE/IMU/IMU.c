/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "IMU.h"
#include "myiic.h"

void IMU_Init(void)
{
	WriteData(Acc_addr,ACC_ret,0xb6);//reset
	WriteData(Acc_addr,ACC_range,0x0c);//+/- 16g
	WriteData(Gyro_addr,GYRO_ret,0xb6);
	WriteData(Gyro_addr,GYRO_range,0x02);// 500
	WriteData(Mag_addr,MAG_ret,0x81);
}
