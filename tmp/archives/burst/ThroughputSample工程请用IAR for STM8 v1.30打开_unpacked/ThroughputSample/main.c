/**
  ******************************************************************************
  * @file project\main.c
  * @brief  厦门卓万电子科技有限公司
  *         ZAuZx_Tx系列串口透传解决方案
  *         低功耗伺服/突发数据上传范例
  *         使用请参考相应手册
  *         样品购买请访问 http://zatech.taobao.com
  * @author ZATech
  * @version V1.0.0
  * @date    08/25/2019
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm8l10x.h"
#include "board.h"

#define USE_FULL_ASSERT

//#define WATCHDOG

#define RESET_ZIGBEE_RATIO  2880  //重启zigbee模块与唤醒次数比例，每RESET_ZIGBEE_TO_WAKE_RATIO次唤醒（一般为30s）重启一次zigbee模块

#define USE_SEC_ADDR    //定义使用地址设置第二方案-串口设置地址，如采用IO口地址设置方式，请注释掉此句

#define READY_REPORT    //定义准备完毕后自动上报本机信息

//#define QUIET_WHILE_SEND  //终端在持续发送数据时是否保存静默，取消注释此句使能静默-激活流程，注释此句则不静默

#define UI_STRING   //定义为字符串输出界面

//初始参数配置，取消注释行来发送相应参数配置项，参数含义请详细参考手册，如无必要请勿随意修改参数。旧版模块不支持部分命令。
const char *Initial_Setting[]=
{
//  "BAUD_RATE 115200",   //波特率，修改波特率后需用新波特率与模块通信，无必要请勿修改
//  "CHANNEL 8192",       //信道
//  "PANID 4372",         //网络ID
//  "TX_POWER 21",        //发射功率
//  "ANNCE_RATIO 100",    //参考手册，不建议修改
//  "UNI_SEC_ADDR -1",    //已在之前通过单片机本机序列号进行设置，如在此设置，将会覆盖之前设置
  "START_WAIT_CYCLE 0",   //改为快速启动入网。注意！！快速启动仅适用串口设置地址的方式。IO口设置地址不应使用快速启动
  "JOIN_DURATION 0",
//  "JOIN_TIMEOUT 50",
//  "SLEEP_TIMEOUT 50",
//  "REJOIN_TIMEOUT 1000",
  "POLL_RATE 3000",       //上电都重新激活，避免异常后陷入死循环
  "RESPONSE_POLL_RATE 100",
//  "QUEUED_POLL_RATE 100",
//  "IS_SRC_ADDR_ADDED -1",
  "PW_RESET 1",   //重启以使参数生效，并使模块正常工作
  0   //请保留此行，程序通过此结束符结束设置命令
};

//将模块置成静默状态，具体含义参考手册
const char *Quiet_Setting[]=
{
  "POLL_RATE 0",
  "RESPONSE_POLL_RATE 0",
  0   //请保留此行，程序通过此结束符结束设置命令
};

//将模块重新激活，具体含义参考手册
const char *Activate_Setting[]=
{
  "POLL_RATE 3000",
  "RESPONSE_POLL_RATE 100",
  "PW_RESET 1",   //当前应用较为复杂，激活后重启一下模块以避免异常
  0   //请保留此行，程序通过此结束符结束设置命令
};

/* Private defines -----------------------------------------------------------*/
volatile u8 i,j,k;
u8  AddrHi,AddrLo;
u16 Addr,SettingTemp;
u8  UARTSendDataBuf[84];
u8  UARTRcvDataBuf[128];  //串口接收数据缓存，包长度不会超过80字节，也不会有多个包同时到达，但是为凑个整数，就设大一点
u8  UARTRcvDataIdx=0;     //接收数据缓存指针
u8  WakeFromData=FALSE;   //是否由接收数据唤醒
u32 ResetZigbeeCount=1;   //跳过=0时的第一次重启

/* Private function prototypes -----------------------------------------------*/
void Delay(u16 nCount);

void Setting_Command(const char *Setting[]);

void Ready_Report(void);

void UART_Send_Data(u8 DataBuf[], u8 DataLength, bool IsWakeSigGen);

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  /*----------IO口设置----------*/
  GPIO_Init(ADDR_LOW_PORT, GPIO_Pin_All, GPIO_Mode_Out_PP_Low_Slow);        //8位地址
  
  GPIO_Init(RST_PORT, RST_PIN, GPIO_Mode_Out_PP_High_Slow);    //控制zigbee模块RST口设置为输出高电平
  
  //保持温湿度底板SHT10与DS18B20引脚定义及初始化，避免连接传感器造成的漏电
  GPIO_Init(SHT10_SDA_PORT, SHT10_SDA_PIN, GPIO_Mode_Out_OD_Low_Fast); 
  GPIO_Init(SHT10_SCL_PORT, SHT10_SCL_PIN, GPIO_Mode_Out_OD_Low_Slow); 
  SHT10_SDA_PORT->ODR |= (uint8_t)SHT10_SDA_PIN;    //输出高电平以免和外部上拉电阻不匹配导致漏电
  SHT10_SCL_PORT->ODR |= (uint8_t)SHT10_SCL_PIN;
  
  //P1.3/mode0 - 模块输入睡眠，stm8输出睡眠
  GPIO_Init( MODE0_PORT, MODE0_PIN, GPIO_Mode_Out_PP_Low_Slow);
  
  //P1.5/mode1 - 模块输出睡眠，stm8输入睡眠
  //Zigbee透传模块输出给stm8的唤醒信号为高电平，并在10ms后开始发送串口信号
  //由于stm8输入无内部下拉选项，因此只能设置为浮动输入
  //当不接Zigbee模块单独进行stm8程序调试时，浮动输入将可能导致持续发生中断，所以请务必接入Zigbee模块或在不接入模块时改为输入上拉
  GPIO_Init( MODE1_PORT, MODE1_PIN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Rising);
  
  GPIO_Init(SENSOR_DATA_PORT, SENSOR_DATA_PIN, GPIO_Mode_Out_PP_High_Fast);   //DS18B20传感器数据口拉高避免漏电
  
  
  /*----------系统周期设置与内部模块使能----------*/
  CLK_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_AWU, ENABLE);      //使能唤醒
  CLK_MasterPrescalerConfig(CLK_MasterPrescaler_HSIDiv8);     //时钟8分频，2MHz
  
  
  /*----------唤醒初始化----------*/
  AWU_DeInit();
  
  
  /*----------串口初始化----------*/
  CLK_PeripheralClockConfig(CLK_Peripheral_USART, ENABLE);   //使能串口
  GPIO_ExternalPullUpConfig(GPIOC,GPIO_Pin_2|GPIO_Pin_3, ENABLE);   //拉高电平
  USART_DeInit();
  USART_Init(115200,                            //波特率115200
            USART_WordLength_8D,                //8位数据位
            USART_StopBits_1,                   //1位停止位
            USART_Parity_No,                    //无校验
            USART_Mode_Rx | USART_Mode_Tx);     //接收和发送使能
  //USART_ITConfig(USART_IT_TXE, ENABLE);         //使能发送中断
  USART_ClearFlag();                            //清空串口接收标志位
  USART_Cmd(ENABLE);    //串口开始工作
  
  
  /*----------看门狗初始化----------*/
  //注意！！看门狗的最长喂狗时限仅1~2秒，而本程序中单片机休眠时间最长设置为30秒
  //所以需要将Option Byte中的OPT4由默认的0x00改为0x01，以使休眠时看门狗暂停
  //Option Byte无法在程序中修改，只能通过烧写软件如STVP在烧写时由SWIM协议外部写入
  //因此在调试时看门狗功能无法实现
  //由SWIM外部烧写时请自行宏定义WATCHDOG以使看门狗生效
#ifdef  WATCHDOG
  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);    //看门狗时限设为最长的1724.63ms
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();
#endif
  
  
  /*----------以下进行Zigbee串口透传模块地址的设置----------*/
  //我们以STM8L 96bit唯一序列号中的8bit作为地址低位，地址高3bit放空为0x000
  
  //有两种设置地址的方式：外部拉地址IO和内部串口设置
  //前者极易造成漏电，需仔细参考手册和温湿度采集范例，不推荐采用
  //推荐采用后者的串口设置方式，将地址IO口全部放空或下拉，写入串口设置地址值后重启模块即可以完成地址设置
  
  AddrHi=0;   //地址高位置0，本范例放空了模块地址高位设置口，故地址高位为0，如需改为自己的程序请记得设置该地址并在下面的程序中设置对应的地址IO
  FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
  FLASH_Unlock(FLASH_MemType_Program);
  //AddrLo=FLASH_ReadByte(0x4926);    //采用X-coordinator
  AddrLo=FLASH_ReadByte(0x4928);    //采用Y-coordinator
  //AddrLo=(FLASH_ReadByte(0x4926)<<4) | (FLASH_ReadByte(0x4928)&0x0F);   //X-coordinator和Y-coordinator各占4bit
  FLASH_Lock(FLASH_MemType_Program);
  Addr = ((u16)AddrHi<<8) | AddrLo; //将高低位地址组合成完整地址
  
  //等待模块初始化完成以能响应串口设置命令
  for(u8 i=0; i<6; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时1秒以上，这里要有足量等待时间，因为模块上电一段时间后串口才开始工作接收数据
  }
  
#ifndef USE_SEC_ADDR    //采用IO口设置方案，该方案通过拉zigbee模块的地址IO来设置地址，此方式可能造成漏电故不推荐采用
  //GPIO_Write(ADDR_HIGH_PORT, AddrHi);   //本程序放空了模块地址高位设置口，所以这里无需再拉IO口，如改为自己的程序请记得设置高位的地址并在此设置IO
  GPIO_Write(ADDR_LOW_PORT, AddrLo);    //写入低8位地址，高3位放空
  for(u8 i=0; i<3; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时0.5秒以上，因为在0.5秒时读入地址IO口，之后地址IO口状态不影响地址设置
  }
  //GPIO_Write(ADDR_HIGH_PORT, 0);
  GPIO_Write(ADDR_LOW_PORT, 0x00);    //将地址IO口置回0以降低功耗
#else   //采用串口设置地址，推荐
  //采用串口设置地址方式需下拉所有地址IO口使之为0x0000
  //本程序IO口初始化时均为下拉低电平，无需更改电平，注意如果拉高任何地址IO口，则串口地址无效，自动采用IO口所设置的地址
  //GPIO_Write(ADDR_LOW_PORT, 0);   //地址低位写入低电平，初始化已为低电平，无需写入
  //GPIO_Write(ADDR_HIGH_PORT, 0);  //地址高位写入低电平，初始化已为低电平，无需写入
  
  //以下将地址转换为串口设置命令字符串，首先输入设置命令UNI_SEC_ADDR和命令与设置值之间的空格
  UARTSendDataBuf[0]='U';UARTSendDataBuf[1]='N';UARTSendDataBuf[2]='I';UARTSendDataBuf[3]='_';UARTSendDataBuf[4]='S';
  UARTSendDataBuf[5]='E';UARTSendDataBuf[6]='C';UARTSendDataBuf[7]='_';UARTSendDataBuf[8]='A';UARTSendDataBuf[9]='D';
  UARTSendDataBuf[10]='D';UARTSendDataBuf[11]='R';UARTSendDataBuf[12]=' ';
  //然后将13bit地址转换为四字节字符串，如0x0789转化为“1929”，0x0089转化为“0137”，并将值填入UARTSendDataBuf[13]~UARTSendDataBuf[16]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[16-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
  //发送设置地址命令
  UART_Send_Data(UARTSendDataBuf, 17, TRUE);
  
  Delay(0x3000);    //稍作等待  
#endif
  
  /*----------以下进行其他参数设置，请在35行处取消需要设置的命令行的注释并设置相应值----------*/
  
  Setting_Command(Initial_Setting);

  //等待模块重启入网
  for(u8 i=0; i<15; i++)
  {
    Delay(0xFFFF);    //每个Delay约200ms，总延时3秒，设备多时建议改长，这里要有足量等待时间，因为模块上电一段时间后才可收发数据，设置过短可能不能成功发送就绪包
  }
    
  
#ifdef  READY_REPORT
  //上报本机就绪信息，上报信息可能会打乱其他设备正常数据收发，建议实际工程中不采用
  Ready_Report();
#endif
    
  disableInterrupts();
  
  /* Infinite loop */
  while (1)
  {
#ifdef  WATCHDOG
    IWDG_ReloadCounter();   //喂狗
#endif
    
    //USART->DR;  //读取以清空串口缓存
    //USART_Cmd(DISABLE); //关闭串口避免串口受干扰
    
    enableInterrupts();
    AWU_Init(AWU_Timebase_30s);   //30秒睡眠，实测睡眠时间误差较大
    AWU_ReInitCounter();
    AWU_Cmd(ENABLE);
    halt();
    disableInterrupts();
    
    if(WakeFromData==TRUE)  //模块P1.5唤醒，开始串口接收数据
    {
      USART_Cmd(DISABLE); //重新开关串口以使DR读取流程正常，也可在238行处读取DR以清空，或将本句改在239行处
      USART_Cmd(ENABLE);  //STM8的串口有点问题，需要这么复杂的操作
      USART->SR;    //读取一次以清空SR与DR缓存，注意这里要按USART->SR接着USART->DR的格式才能清空SR的USART_SR_IDLE位
      USART->DR;    //如不进行清空，则会发生串口每次发送完数据后都不能正常接收第一个包
      
      UARTRcvDataIdx=0;   //指针归零
      while(!(USART->SR & USART_SR_IDLE))   //采用IDLE标志位收包，IDLE出现说明包结束
      {
        if(USART->SR & USART_SR_RXNE)   //串口接收非空
        {
          //串口接收，包长不会超过80字节，也不会同时收到多个包，但保险起见防止溢出仍然做成循环队列
          UARTRcvDataBuf[(UARTRcvDataIdx++)%128]=USART->DR;
        }
      }
      //经大量测试，收到的包都是完整正确的，但在十分繁忙时可能会无数据触发，暂不需理会，只需忽略包长度为0的情况即可
      //这可能源于之前未重置串口，以及未清空SR的USART_SR_IDLE位，导致判为空包；此版本已在250行处做改进，应该不会再出现丢包
      if(UARTRcvDataIdx!=0)
      {
        //此时已在UARTRcvDataBuf缓存内收到包长度为UARTRcvDataIdx的数据包，在此进行数据包处理
        
        //本范例的流程，主机广播发送一轮或多轮（防止终端漏收丢包故需多轮，每轮间隔根据POLL_RATE定为3秒）采集包给所有终端，告知需采集的终端地址
        //每个终端在收到采集包后比对采集终端的地址与本机地址，并做如下操作：
        //a.与本机地址不符，则将本机置为静默状态以免干扰采集设备发送数据，并在足够长的时间后恢复激活状态
        //b.与本机相符，则将本机置为静默状态，并在等待一轮或多轮（每轮根据POLL_RATE定为3秒），等其他所有设备均进入静默状态后开始全速发送数据
        //  并在数据发送完毕后立即恢复激活状态等待主机确认
        //主机在收到上报数据包后校验是否正确，如不正确则再次原样发送采集包，则本机将会再次原样进行一次数据上报流程
        //如正确，则主机等待所有设备自动恢复激活状态后进行下一次采集
          
        //在此我们进行这样的演示：主机发送0xAA 0xFF 0xFF 0x55 0xA3 地址高位 地址低位 0x3A采集对应地址的终端设备数据
        //所有终端都会收到0xA3 址高位 地址低位 0x3A的采集数据包，而后进行比对自身地址与进一步操作
        
        if(UARTRcvDataIdx==4 && UARTRcvDataBuf[0]==0xA3 && UARTRcvDataBuf[3]==0x3A)   //收到采集包
        {
          //判断是否是自身地址
          if(UARTRcvDataBuf[1]!=AddrHi || UARTRcvDataBuf[2]!=AddrLo)
          {
            //非本机地址，进入静默状态，并在一段时间后自动回复激活状态
            Setting_Command(Quiet_Setting);
            for(i=0; i<2; i++)  //这里示范进入1分钟静默时间，之后重新激活
            {
              enableInterrupts();
              AWU_Init(AWU_Timebase_30s);   //30秒睡眠，实测睡眠时间误差较大
              AWU_ReInitCounter();
              AWU_Cmd(ENABLE);
              halt();
              disableInterrupts();
            }
            Setting_Command(Activate_Setting);  //恢复激活
            for(u8 i=0; i<15; i++)
            {
              Delay(0xFFFF);    //每个Delay约200ms，总延时3秒等待模块就绪
            }
#ifdef  READY_REPORT
            //上报本机就绪信息，上报信息可能会打乱其他设备正常数据收发，建议实际工程中不采用
            Ready_Report();
#endif
          }
          else
          {
            //本机地址，进入静默状态，并等待一轮或多轮（每轮根据POLL_RATE定为3秒），等其他所有设备均进入静默状态后开始全速发送数据
            
            //实际测试，终端在高速发送数据时可以不静默，不影响发包，本范例未静默，如需改动，可使能QUIET_WHILE_SEND的定义
#ifdef  QUIET_WHILE_SEND
            Setting_Command(Quiet_Setting);
#endif
            for(i=0; i<3; i++)  //这里示范假设主机会广播两轮采集包，故等待6秒
            {
              enableInterrupts();
              AWU_Init(AWU_Timebase_2s);
              AWU_ReInitCounter();
              AWU_Cmd(ENABLE);
              halt();
              disableInterrupts();
            }
            //开始全速发送数据
            GPIO_WriteBit(MODE0_PORT, MODE0_PIN, SET);    //唤醒串口
            Delay(0x400);         //等待3ms
            
            //测试发送1000个80字节包，共80K字节
            j=0;
            for(u32 k=0; k<1000; k++)
            {
              j=(j+1)%10;
              UARTSendDataBuf[0]=0xAA;UARTSendDataBuf[1]=0x00;UARTSendDataBuf[2]=0x00;UARTSendDataBuf[3]=0x55;  //发给主机的包头
              UARTSendDataBuf[4]=(u8)(k>>8);UARTSendDataBuf[5]=(u8)(k&0x00FF);  //数据包编号
              for(i=6;i<84;i++)
              {
                UARTSendDataBuf[i]=j+'0';
              }
              UART_Send_Data(UARTSendDataBuf, 84, FALSE);
              //等待T0后发下一个包，每个count对应3us
              //根据手册，包间隔要求15毫秒，对应5000个count，每22毫秒可发送一个80字节包（3.6KB/s），信道良好时可保持0误包率
              //测试时可适当调小，实测调至1500时仍可保持千分之一以下误包率，此时每12毫秒即可发送一个80字节包（6.7KB/s）
              Delay(5000);
            }
            GPIO_WriteBit(MODE0_PORT, MODE0_PIN, RESET);    //释放唤醒信号
            
            //恢复激活，此时如有数据错误，主机可等待三秒终端就绪后再次发送采集包，本机会重新进行发送流程
            //如终端未进行静默-激活流程（本范例就未静默），则主机可不做等待直接发送采集包或其他包与本机进行通信
            //也可自行设计流程，重传丢失包
#ifdef  QUIET_WHILE_SEND
            Setting_Command(Activate_Setting);
#endif
          }
        }
        //数据包处理完毕
      }
    }
    
    
    if(ResetZigbeeCount==0)
    {
      //等待zigbee发完最后一包数据后抽空重启
      enableInterrupts();
      AWU_Init(AWU_Timebase_128ms);
      AWU_ReInitCounter();
      AWU_Cmd(ENABLE);
      halt();
      disableInterrupts();
      
      GPIO_WriteBit(RST_PORT, RST_PIN, RESET);    //拉低重启zigbee模块
      DELAY_5US();
      GPIO_WriteBit(RST_PORT, RST_PIN, SET);      //复原
    }
    ResetZigbeeCount++;
    ResetZigbeeCount = ResetZigbeeCount % RESET_ZIGBEE_RATIO;
    
  }

}

/**
  * @brief  Delay.
  * @param  nCount
  * @retval None
  */
void Delay(u16 nCount)   //实测每个count对应6指令周期，2MHz下对应3us
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}


/**
  * @brief  zigbee参数设置.
  * @param  Setting - 参数列表
  * @retval None
  */
void Setting_Command(const char *Setting[])
{
  i=0;
  while(Setting[i][0]!=0)   //到达参数命令行结尾的0后设置完成
  {
    j=0;
    while(Setting[i][j]!=0) //每条命令字符串结尾的\0处后停止
    {
      UARTSendDataBuf[j]=Setting[i][j];   //复制命令字符串
      j++;
    }
    UART_Send_Data(UARTSendDataBuf, j, TRUE);   //发送设置命令，单条单次唤醒
    Delay(0x3000);   //稍作等待
    i++;
  }
    
  USART_ClearFlag();  //注意！！设置参数时都是有回包的，虽然我们忽略了这些回包，但还是要清空标志位
}


/**
  * @brief  报告本机就绪.
  * @param  None
  * @retval None
  */
void Ready_Report(void)
{
#ifndef UI_STRING
  //HEX格式：主机收到0xC3 地址高位 地址低位 0x3C的本机报告包表示本机已准备完毕
  UARTSendDataBuf[0]=0xAA;      //4字节目的地址包头，0xAA 目的地址高位 目的地址低位 0x55，主机目的地址高低位均为0x00
  UARTSendDataBuf[1]=0x00;
  UARTSendDataBuf[2]=0x00;
  UARTSendDataBuf[3]=0x55;
  UARTSendDataBuf[4]=0xC3;
  UARTSendDataBuf[5]=AddrHi;    //发送给主机本机地址以方便主机的寻址，地址高位
  UARTSendDataBuf[6]=AddrLo;    //地址低位
  UARTSendDataBuf[7]=0x3C;
  UART_Send_Data(UARTSendDataBuf, 8, TRUE);         //将数据发送到主机
#else
  //字符串格式
  //格式 ID:1234 READY
  //0~3，AA XX XX 55，目的地址
  //4~6, 3字节头
  //7~10, 4字节地址
  //11~16, 6字节尾
  UARTSendDataBuf[0]=0xAA;UARTSendDataBuf[1]=0x00;UARTSendDataBuf[2]=0x00;UARTSendDataBuf[3]=0x55;
  UARTSendDataBuf[4]='I';UARTSendDataBuf[5]='D';UARTSendDataBuf[6]=':';
  //将本机地址转换为十字节字符串，并将值填入UARTSendDataBuf[10]~UARTSendDataBuf[19]
  SettingTemp=Addr;
  for(u8 i=0; i<4; i++)
  {
    UARTSendDataBuf[10-i] = '0' + (SettingTemp % 10);
    SettingTemp /= 10;
  }
/*  //将头部的所有0替换为空格
  i=7;
  while(UARTSendDataBuf[i]=='0')
  {
    UARTSendDataBuf[i++]=' ';
  }
*/  
  UARTSendDataBuf[11]=' ';UARTSendDataBuf[12]='R';UARTSendDataBuf[13]='E';UARTSendDataBuf[14]='A';UARTSendDataBuf[15]='D';UARTSendDataBuf[16]='Y';
  
  UART_Send_Data(UARTSendDataBuf, 17, TRUE);         //将数据发送到主机
#endif
}


/**
  * @brief  发送给模块数据或命令
  * @param  DataBuf - 发送给模块的数据，DataLength - 发送长度，IsWakeSigGen - 是否生成唤醒信号
  * 终端发数据都是要唤醒的
  * IsWakeSigGen参数为TRUE时，每次都会在本函数里自动生成一个完整的唤醒信号，并在发送完之后释放，建议单条发送数据包时采用
  * 而连续大量发包时，建议模块全程唤醒，也就是在本函数外部单独拉醒模块，而后多次调用本函数并设置IsWakeSigGen参数为FALSE，发包结束后再外部释放唤醒信号
  * @retval None
  */
void UART_Send_Data(u8 DataBuf[], u8 DataLength, bool IsWakeSigGen)
{
  u8 idx=0;
  
  USART_Cmd(ENABLE);    //使能串口
  
  if(IsWakeSigGen)
  {
    //选择如下两种唤醒方式之一
    
    //第一种，IO口唤醒，将唤醒口拉高后等待3ms，开始发送串口信号
    //在发送完一个数据包后等待20ms以上发送下一个数据包，在所有串口信号发送完毕后拉低唤醒口
    GPIO_WriteBit(MODE0_PORT, MODE0_PIN, SET);    //唤醒串口
    Delay(0x400);         //等待3ms
    
    //第二种，串口直接唤醒，发送一字节0x55唤醒串口后等待10ms发送串口数据，在发送完一个数据包后等待20~150ms发送下一个数据包
    //如果超过150ms未发送数据包，则需要重新进行唤醒
    //USART_SendData8(0x55);    //发送唤醒字节
    //while((USART->SR & 0x80) == 0);      //等待发送缓存空
    //Delay(0xD00);         //等待10ms
  }
  
  for(idx=0; idx<DataLength; idx++)
  {
    USART_SendData8(DataBuf[idx]);    //发送当前字符
    while((USART->SR & 0x80) == 0);   //等待发送缓存空
  }
  
  while((USART->SR & 0x40) == 0);       //等待发送完毕
  
  if(IsWakeSigGen)
  {
    GPIO_WriteBit(MODE0_PORT, MODE0_PIN, RESET);    //释放唤醒信号，让模块重新进入睡眠
  }
  
  //USART_Cmd(DISABLE);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/