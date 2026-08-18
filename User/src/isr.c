#include "include.h"
#include "isr.h"
#include "Attitude_Calculation.h"
#include "DataStore.h"


vuint8                 IsGyroOffsetReset = 0;     /////如果需要进行陀螺仪零飘矫正则将改变量置为   1
/////置1的方式可以通过 按键或者别的操作
BMX055Datatypedef      BMX055_data;
EulerAngleTypedef      SystemAttitude;            /////姿态角
EulerAngleTypedef      SystemAttitudeRate;        /////姿态角速度
AttitudeDatatypedef    GyroOffset;


void GyroOffset_init(void)      /////////陀螺仪零飘初始化
{
  static uint16 Count = 0;
  if(Count == 0)
  {
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
  }
  
  if(Count == 1000)
  {
    GyroOffset.Xdata /= 1000;
    GyroOffset.Ydata /= 1000;
    GyroOffset.Zdata /= 1000;
    IsGyroOffsetReset = 0;
    DataSave();
    Count = 0;
  }
  else
  {
    BMX055_DataRead(&BMX055_data, 0);
    GyroOffset.Xdata += BMX055_data.GYROXdata;
    GyroOffset.Ydata += BMX055_data.GYROYdata;
    GyroOffset.Zdata += BMX055_data.GYROZdata;
    Count++;
  }
}


/**************************************************/
/*将全部的控制及采集函数放于该中断*/
/*************************************************/
//float TestUsing = 0;   //在K66 210M  有PFU的情况下，四元数姿态解算加姿态数据采集共需要 280ns
volatile bool IsSend = false;
float AccZAngle = 0;
void PIT0_IRQHandler(void)
{
  static uint8 MAGWaitCount = 0;
  static bool IsAttitudeinit = false;
  static uint8 SendCount = 0;
  PIT_FlAG_CLR(PIT0);
  
  PIT_time_start(PIT2);
  if(IsSend == false)
  {
    SendCount++;
    if(SendCount == 5)
    {
      IsSend = true;
      SendCount = 0;
    }
  }
  if(IsGyroOffsetReset)
  {
    GyroOffset_init();             ///////零偏矫正过程中不进行姿态结算，并保证姿态模块静止
    return;
  }
  /*****************************************************************/
  /*我是分界线，上面是零偏矫正*/
  /******************************************************************/
  
  MAGWaitCount++;
  if(MAGWaitCount < 15)
    BMX055_DataRead(&BMX055_data, 0);
  else
  {
    BMX055_DataRead(&BMX055_data, 1);     //////每30ms读取一次磁力计
    MAGWaitCount = 0;
  }
  BMX055_data.GYROXdata = (BMX055_data.GYROXdata - GyroOffset.Xdata) * 0.030517578;   
  BMX055_data.GYROYdata = (BMX055_data.GYROYdata - GyroOffset.Ydata) * 0.030517578;
  BMX055_data.GYROZdata = (BMX055_data.GYROZdata - GyroOffset.Zdata) * 0.030517578;
  ///////1000 / 32768     //////BMX055本身零飘几乎可以忽略不计，但是安全起见还是矫正一下
  BMX055_data.ACCXdata *= 0.001953125;    ///////4 / 2048
  BMX055_data.ACCYdata *= 0.001953125;
  BMX055_data.ACCZdata *= 0.001953125;
  
  
  Acc.Xdata = BMX055_data.ACCXdata;
  Acc.Ydata = BMX055_data.ACCYdata;
  Acc.Zdata = BMX055_data.ACCZdata;
  Gyro.Xdata = BMX055_data.GYROXdata;
  Gyro.Ydata = BMX055_data.GYROYdata;
  Gyro.Zdata = BMX055_data.GYROZdata;

  
  if(IsAttitudeinit == false)
  {
    Quaternion_init();                    ////姿态解算初始化        
    IsAttitudeinit = true;
  }
  else
  {
    Attitude_UpdateGyro();                /////快速更新
    Attitude_UpdateAcc();                 /////深度融合更新
    
    
    SystemAttitude.Pitch = -EulerAngle.Roll / PI * 180;            ////转为角度   俯仰角
    SystemAttitude.Roll = EulerAngle.Pitch / PI * 180;             ////翻滚角
    SystemAttitude.Yaw = EulerAngle.Yaw / PI * 180;                ////偏航角
    SystemAttitudeRate.Pitch = -EulerAngleRate.Roll / PI * 180;   ////俯仰角速度
    SystemAttitudeRate.Yaw = EulerAngleRate.Yaw / PI * 180;       ////偏航角速度
    
    float AccZ, AccZAdjust;    
    AccZ = -Acc.Zdata;
    if (AccZ > 1)
	AccZ = 1;
    if (AccZ < -1)
	AccZ = -1;            
    AccZAngle = asinf(AccZ) * 180 / PI;
    AccZAdjust = (AccZAngle - SystemAttitude.Pitch);
    SystemAttitude.Pitch += (-Gyro.Xdata + AccZAdjust) * PERIODS;
  }
  //TestUsing = PIT_time_get(PIT2);
  ///////////////至于磁力计只用于起跑线检测则无需进行浮点运算，只要判断数值大小即可
  
  
  /*****************************************************************/
  /*我是分界线，下面是姿态控制*/
  /******************************************************************/
  
  
  
  /*****************************************************************/
  /*我是分界线，上面是姿态控制*/
  /******************************************************************/
}
