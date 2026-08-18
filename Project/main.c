#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "UART1.h"
#include "UART2.h"
#include "kalman.h"	
#include "IOI2C.h"
#include "wit_c_sdk.h"
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define QUAT_UPDATE		0x10
#define READ_UPDATE		0x80
#define REQUIRED_UPDATE QUAT_UPDATE
#define SOURCE_FLAG_METADATA_VALID      0x01u
#define SOURCE_FLAG_HARDWARE_TIME_VALID 0x02u
#define SOURCE_FLAG_CLOCK_72MHZ         0x04u
#ifndef DEVICE_LOGICAL_ID
#define DEVICE_LOGICAL_ID 0x03
#endif
#if (DEVICE_LOGICAL_ID < 1) || (DEVICE_LOGICAL_ID > 9)
#error DEVICE_LOGICAL_ID must be in the range 1..9
#endif
static volatile unsigned char s_cDataUpdate = 0;
static volatile char s_cCmd = 0xff;
static uint32_t s_uiFrameSequence = 0;
static volatile uint32_t s_uiSourceTimerOverflows = 0;
static uint32_t s_uiSourceTimerHz = 1000000u;
static uint8_t s_ucSourceFlags = SOURCE_FLAG_METADATA_VALID;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

void TIM4_Init(u16 pre, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    TIM_TimeBaseInitStructure.TIM_Period=pre;
    TIM_TimeBaseInitStructure.TIM_Prescaler=psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);

    //�����ж��źţ�����ж��źŲ����жϡ�
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    //�������ȼ�
    NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //������ʱ��
    TIM_Cmd(TIM4,ENABLE);

}

/*
 * TIM4 provides the source-time clock carried by every V2 frame. Its
 * prescaler is derived from the clock tree that is actually running, so an
 * HSE/PLL startup failure cannot turn 1 second into roughly 9 seconds.
 */
static void SourceClock_Init(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock_hz;
    uint32_t prescaler_div;

    RCC_GetClocksFreq(&clocks);
    timer_clock_hz = clocks.PCLK1_Frequency;
    if (((RCC->CFGR >> 8) & 0x07u) >= 4u)
        timer_clock_hz *= 2u;

    prescaler_div = timer_clock_hz / 1000000u;
    if (prescaler_div == 0u)
        prescaler_div = 1u;
    s_uiSourceTimerHz = timer_clock_hz / prescaler_div;

    TIM4_Init(0xffffu, (u16)(prescaler_div - 1u));
    s_uiSourceTimerOverflows = 0u;
    s_ucSourceFlags |= SOURCE_FLAG_HARDWARE_TIME_VALID;
    if ((SystemCoreClock >= 64000000u) && (SystemCoreClock <= 80000000u))
        s_ucSourceFlags |= SOURCE_FLAG_CLOCK_72MHZ;
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        s_uiSourceTimerOverflows++;
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

static uint32_t ReadSourceTickMs(void)
{
    uint32_t overflows;
    uint16_t counter;
    uint64_t source_ticks;

    __disable_irq();
    overflows = s_uiSourceTimerOverflows;
    counter = (uint16_t)TIM_GetCounter(TIM4);
    if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)
    {
        overflows++;
        counter = (uint16_t)TIM_GetCounter(TIM4);
    }
    __enable_irq();

    source_ticks = ((uint64_t)overflows << 16) | counter;
    return (uint32_t)((source_ticks * 1000u) / s_uiSourceTimerHz);
}

float fAcc[3], fGyro[3], fAngle[3],my_q[4];
int i;
void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}
static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart1Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart1Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG|RSW_Q) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}
void Uart2Send(unsigned char *p_data, unsigned int uiSize)
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, *p_data++);		
	}
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}

static uint32_t ReadStableHardwareId(void)
{
    const volatile uint32_t *uid = (const volatile uint32_t *)0x1FFFF7E8;
    uint32_t hash = 2166136261u;
    int i;
    for (i = 0; i < 3; i++)
    {
        hash ^= uid[i];
        hash *= 16777619u;
    }
    return hash == 0 ? 1u : hash;
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AX:
            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GX:
            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;// 更新角速度数据状态（设置 GYRO_UPDATE 标志位）
            break;
            case HX:
            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case q0:
            case q1:
            case q2:
            case q3:
				s_cDataUpdate |= QUAT_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart1Init(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			delay_ms(100);
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}


int main(void)
{
    uint32_t hardware_id;
    uint32_t system_clock_mhz;
   
    SystemCoreClockUpdate();
    system_clock_mhz = SystemCoreClock / 1000000u;
    if (system_clock_mhz == 0u)
        system_clock_mhz = 1u;
    SysTick_Init((u8)system_clock_mhz);
    SourceClock_Init();
    Initial_UART2(115200);   
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
 
    
    // 
    AutoScanSensor();
    if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK)
    {
        printf("\r\nSet fixed 10Hz output rate error\r\n");
    }
    if(WitSetContent(RSW_Q) != WIT_HAL_OK)
    {
        printf("\r\nSet RSW Error\r\n");
    }
    s_cDataUpdate = 0;
    hardware_id = ReadStableHardwareId();
 
    while (1)
    {
        Delayms(50); // 20Hz
        CmdProcess(); 
 
        if ((s_cDataUpdate & REQUIRED_UPDATE) == REQUIRED_UPDATE) 
        {
            int16_t raw_q0, raw_q1, raw_q2, raw_q3;

            __disable_irq();
            raw_q0 = sReg[q0];
            raw_q1 = sReg[q1];
            raw_q2 = sReg[q2];
            raw_q3 = sReg[q3];
            s_cDataUpdate &= (unsigned char)~REQUIRED_UPDATE;
            __enable_irq();

            // 读取四元数数据
            float w  = raw_q0 / 32768.0f;
            float x  = raw_q1 / 32768.0f;
            float y  = raw_q2 / 32768.0f;
            float z  = raw_q3 / 32768.0f;
            
            s_uiFrameSequence++;
            UART2_siyuan_v2(DEVICE_LOGICAL_ID,
                            hardware_id,
                            s_uiFrameSequence,
                            ReadSourceTickMs(),
                            s_ucSourceFlags,
                            w, x, y, z);
        }
    }
}






