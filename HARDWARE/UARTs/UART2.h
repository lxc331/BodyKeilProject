#ifndef __UART2_H
#define __UART2_H

#include <stdio.h>
#include "stm32f10x.h"

#define Gyro_init  0xE0
#define High_init  0xE2

#define UART2_LINK_COMMAND_CONFIG_SYNC 0x01u
#define UART2_LINK_COMMAND_PAUSE       0x02u

typedef struct
{
    uint8_t Command;
    uint8_t TransmitRateHz;
    uint32_t SyncToken;
} UART2_LinkCommand;


void Initial_UART2(u32 baudrate);
void UART2_Put_Char(unsigned char DataToSend);
u8 UART2_Get_Char(void);
void UART2_Put_String(unsigned char *Str);
void UART2_Putc_Hex(uint8_t b);
void UART2_Putw_Hex(uint16_t w);
void UART2_Putdw_Hex(uint32_t dw);
void UART2_Putw_Dec(uint32_t w);
void UART2_Putint_Dec(int16_t in);
void UART2_Putintp_Dec(int16_t in);
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
void UART2_Oula(float num0,float num1,float num2);
void UART2_siyuan(uint8_t device_id, float w, float x, float y, float z);
void UART2_norm(float m);
void UART2_siyuan_accel(uint8_t device_id,
                        float w, float x, float y, float z,
                        float ax, float ay, float az,
                        float gx, float gy, float gz);
void UART2_siyuan_v2(uint8_t device_id,
                     uint32_t hardware_id,
                     uint32_t sequence,
                     uint32_t sender_tick_ms,
                     uint8_t source_flags,
                     float w, float x, float y, float z);
uint8_t UART2_TryReadLinkCommand(UART2_LinkCommand *command);
uint32_t UART2_GetLinkRxOverflowCount(void);
uint32_t UART2_GetLinkInvalidFrameCount(void);
#endif

//------------------End of File----------------------------

