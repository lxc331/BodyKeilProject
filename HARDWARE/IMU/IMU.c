/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "IMU.h"
#include "myiic.h"

float exInt1, eyInt1, ezInt1;  // 误差积分
float q0, q1, q2, q3; // 全局四元数
float integralFBhand,handdiff;
uint32_t lastUpdate, now; // 采样周期计数 单位 us
int16_t accgyroval[6];
uint8_t buffer[14];
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
int16_t  HMC5883_FIFO[3][11]; //磁力计滤波
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);
float angles[3];
u8	Gyro[6];
int j; 
int IsGyroOffsetReset = 1;     /////如果需要进行陀螺仪零飘矫正则将改变量置为   1
AttitudeDatatypedef    GyroOffset;
AttitudeDatatypedef    MagOffset;
float Gyro_offset[3];
void GyroOffset_init(void)      /////////陀螺仪零飘初始化
{
  static unsigned short int Count = 0;
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
    Count = 0;
  }
  else
  {
		ReadData(Gyro_addr,GYRO_XL,Gyro,6);//16位
    for( j = 0; j < 3; j ++) //0 1 2  0 1 2 3 4 5
    {
        Gyro_offset[j]=(float)((int16_t)((Gyro[2*j+1] << 8) | Gyro[2*j]));
    }
    GyroOffset.Xdata += Gyro_offset[0];
    GyroOffset.Ydata += Gyro_offset[1];
    GyroOffset.Zdata += Gyro_offset[2];
    Count++;
  }
}
void IMU_Init(void)
{
	int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
    WriteData(Acc_addr,ACC_ret,0xb6);//reset
    WriteData(Acc_addr,ACC_range,0x03);//+/- 2g
    WriteData(Acc_addr,0x10,0x0F);//ODR = 1000 Hz(0x07)
    WriteData(Acc_addr,0x11,0x00);//Normal mode, Sleep duration = 2ms(0x00)
    WriteData(Gyro_addr,GYRO_ret,0xb6);
    WriteData(Gyro_addr,GYRO_range,0x01);// 1000
    WriteData(Gyro_addr,0x10,0x02);// 1000Hz
    WriteData(Gyro_addr,0x11,0x00);// Normal mode, Sleep duration = 2ms(0x00)
    WriteData(Gyro_addr,0x13,0x08);
    WriteData(Mag_addr,MAG_ret,0x82);
		WriteData(Mag_addr,MAG_ret,0x01);
    WriteData(Mag_addr,0x4c,0x00);// Normal Mode, ODR = 10 Hz
    WriteData(Mag_addr,0x51,0x04);// No. of Repetitions for X-Y Axis = 9(0x04)
    WriteData(Mag_addr,0x52,0x16);// No. of Repetitions for Z-Axis = 15(0x0F)
}

void Initial_Timer3(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    /* TIM2 configuration*/
    /* Time Base configuration 基本配置 配置定时器的时基单元*/
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
    /* Disable the TIM2 Update event */
    TIM_UpdateDisableConfig(TIM2, ENABLE);
    /* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
    /* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
    /* Use the External Clock as TIM2 Slave Mode */
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
    /* Enable the TIM2 Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    /* 定时器配置:
    1.设置定时器最大计数值 50000
    2.设置时钟分频系数：TIM_CKD_DIV1
    3. 设置预分频：  1Mhz/50000= 1hz
    4.定时器计数模式  向上计数模式
    */
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 的时钟
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //应用配置到TIM3
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    // 使能TIM3重载寄存器ARR
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
    /* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
    /* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    /* Enable the TIM3 Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
    //启动定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void IMU_init_1(void)
{
    IMU_Init();
    delay_ms(50);
    IMU_Init();
    Initial_Timer3();
    q0 = 1.0f;  //初始化四元数
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt1 = 0.0;
    eyInt1 = 0.0;
    ezInt1 = 0.0;
		MagOffset.Xdata=-16;
		MagOffset.Ydata=7;
		MagOffset.Zdata=5;
    lastUpdate = micros();//更新时间
    now = micros();
}
// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x)
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
    uint32_t temp=0 ;
    temp = TIM2->CNT;//读高16位时间
    temp = temp<<16;
    temp += TIM3->CNT;//读低16位时间
    return temp;
}
/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 20.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.8f   // integral gain governs rate of convergence of gyroscope biases

float q[4];
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
		float norm;

    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    now = micros();  //读取时间
    if(now<lastUpdate)
		{ //定时器溢出过了。
        halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
    }
    else
		{
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间

    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    norm = invSqrt(mx*mx + my*my + mz*mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    // 计算地球磁场的参考方向 
    hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    // 估计的重力和通量方向
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

    // 误差是场的参考方向和传感器测量的方向之间的叉积之和 
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);



    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt1 = exInt1 + ex * Ki * halfT;
        eyInt1 = eyInt1 + ey * Ki * halfT;
        ezInt1 = ezInt1 + ez * Ki * halfT;

        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt1;
        gy = gy + Kp*ey + eyInt1;
        gz = gz + Kp*ez + ezInt1;
    }

    // 调整后的陀螺仪测量 
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 归一化四元数 
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
		
		q[0] = q0; //返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
		
		angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
}

