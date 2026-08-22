/* UART2.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-04-25
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
UART1ͨ�� ��API
------------------------------------
 */

#include "UART2.h"

#define UART2_LINK_RX_BUFFER_SIZE 64u
#define UART2_LINK_RX_BUFFER_MASK (UART2_LINK_RX_BUFFER_SIZE - 1u)
#define UART2_LINK_MAX_PAYLOAD 8u
#define UART2_LINK_MAX_FRAME_SIZE (7u + UART2_LINK_MAX_PAYLOAD)
#define UART2_LINK_PROTOCOL_VERSION 1u

static volatile uint8_t s_ucLinkRxBuffer[UART2_LINK_RX_BUFFER_SIZE];
static volatile uint8_t s_ucLinkRxWriteIndex = 0u;
static volatile uint8_t s_ucLinkRxReadIndex = 0u;
static volatile uint32_t s_uiLinkRxOverflowCount = 0u;
static uint32_t s_uiLinkInvalidFrameCount = 0u;
static uint8_t s_ucLinkFrame[UART2_LINK_MAX_FRAME_SIZE];
static uint8_t s_ucLinkFrameIndex = 0u;
static uint8_t s_ucLinkExpectedFrameSize = 0u;

u8 U2TxBuffer[258];
u8 U2TxCounter=0;
u8 U2count=0; 

void U2NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
          /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_UART2(u32 baudrate)
*��������:		��ʼ��STM32-SDK�������ϵ�RS232�ӿ�
���������
		u32 baudrate   ����RS232���ڵĲ�����
���������û��	
*******************************************************************************/
void Initial_UART2(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ʹ�� UART2 ģ���ʱ��  ʹ�� UART2��Ӧ�����Ŷ˿�PA��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  

  	 /* ����UART2 �ķ�������
	 ����PA9 Ϊ�������  ˢ��Ƶ��50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  ����UART2 �Ľ�������
	  ����PA10Ϊ�������� 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 
	  UART2������:
	  1.������Ϊ���ó���ָ�������� baudrate;
	  2. 8λ����			  USART_WordLength_8b;
	  3.һ��ֹͣλ			  USART_StopBits_1;
	  4. ����żЧ��			  USART_Parity_No ;
	  5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
	  6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Ӧ�����õ�UART2
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);        
    USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//ʹ�ܽ����ж�
	//����UART2
  	USART_Cmd(USART2, ENABLE);
	U2NVIC_Configuration();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART2_Put_Char(unsigned char DataToSend)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, (unsigned char) DataToSend);

//	U2TxBuffer[U2count++] = DataToSend;  
//    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 UART2_Get_Char(void)
*��������:		RS232����һ���ֽ�  һֱ�ȴ���ֱ��UART2���յ�һ���ֽڵ����ݡ�
���������		 û��
���������       UART2���յ�������	
*******************************************************************************/
u8 UART2_Get_Char(void)
{
	while (!(USART2->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART2));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Put_String(unsigned char *Str)
*��������:		RS232�����ַ���
���������
		unsigned char *Str   Ҫ���͵��ַ���
���������û��	
*******************************************************************************/
void UART2_Put_String(unsigned char *Str)
{
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str){
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')UART2_Put_Char(0x0d);
		else if(*Str=='\n')UART2_Put_Char(0x0a);
			else UART2_Put_Char(*Str);
	//�ȴ��������.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}
/*
	//�ж�Strָ��������Ƿ���Ч.
	while(*Str){
	//�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
	if(*Str=='\r')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//�ȴ��������.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//ָ��++ ָ����һ���ֽ�.
	Str++;
	}		 */
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putc_Hex(uint8_t b)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֽ�����
				�Ƚ�Ŀ���ֽ����ݸ�4λת��ASCCII �����ͣ��ٽ���4λת��ASCII����
				��:0xF2 ������ " F2 "
���������
		uint8_t b   Ҫ���͵��ֽ�
���������û��	
*******************************************************************************/
void UART2_Putc_Hex(uint8_t b)
{
      /* �ж�Ŀ���ֽڵĸ�4λ�Ƿ�С��10 */
    if((b >> 4) < 0x0a)
        UART2_Put_Char((b >> 4) + '0'); //С��10  ,����Ӧ����0-9��ASCII
    else
        UART2_Put_Char((b >> 4) - 0x0a + 'A'); //���ڵ���10 ����Ӧ���� A-F

    /* �ж�Ŀ���ֽڵĵ�4λ �Ƿ�С��10*/
    if((b & 0x0f) < 0x0a)
        UART2_Put_Char((b & 0x0f) + '0');//С��10  ,����Ӧ����0-9��ASCII
    else
        UART2_Put_Char((b & 0x0f) - 0x0a + 'A');//���ڵ���10 ����Ӧ���� A-F
   UART2_Put_Char(' '); //����һ���ո�,�����ֿ������ֽ�
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Hex(uint16_t w)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����һ���ֵ�����.���Ƿ���һ��int
				��:0x3456 ������ " 3456 "
���������
		uint16_t w   Ҫ���͵���
���������û��	
*******************************************************************************/
void UART2_Putw_Hex(uint16_t w)
{
	//���͸�8λ����,����һ���ֽڷ���
    UART2_Putc_Hex((uint8_t) (w >> 8));
	//���͵�8λ����,����һ���ֽڷ���
    UART2_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putdw_Hex(uint32_t dw)
*��������:		RS232��ʮ������ASCII��ķ�ʽ����32λ������.
				��:0xF0123456 ������ " F0123456 "
���������
		uint32_t dw   Ҫ���͵�32λ����ֵ
���������û��	
*******************************************************************************/
void UART2_Putdw_Hex(uint32_t dw)
{
    UART2_Putw_Hex((uint16_t) (dw >> 16));
    UART2_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Putw_Dec(uint16_t w)
*��������:		RS232��ʮ����ASCII��ķ�ʽ����16λ������.
				��:0x123 ����������ʮ�������� " 291 "
���������
		uint16_t w   Ҫ���͵�16λ����ֵ
���������û��	
*******************************************************************************/
void UART2_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART2_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}

void UART2_Putint_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART2_Put_Char('-');
	}
   UART2_Putw_Dec(in);
}

void UART2_Putintp_Dec(int16_t in)
{
	if(in<0){
	in=-in;
	UART2_Put_Char('-');
	}
   UART2_Putw_Dec(in/10);
   UART2_Put_Char('.');
   UART2_Putw_Dec(in%10);
}
void float_char(float f,unsigned char *s)
{ 
	union change   
	{      
		float d;

     unsigned char dat[4];   
  	}r1;

		r1.d = f;

    *s = r1.dat[0];

    *(s+1) = r1.dat[1];

    *(s+2) = r1.dat[2];

    *(s+3) = r1.dat[3]; 
}
void UART2_norm(float m)
{
	unsigned char tbuf0[4];
	unsigned char *p = (unsigned char*)&m + 3;//ָ��p��ָ��float������ֽ�
	//��ȡ��Ӧ��4���ֽڣ��ӵ�λ����λ����ʱ�Ϳ�������hex��ʽ�����ݴ�����
	tbuf0[0] = *(p-3);
	tbuf0[1] = *(p-2);
	tbuf0[2] = *(p-1);
	tbuf0[3] = *p;
	UART2_Put_Char(0xFA);

	UART2_Put_Char(0xAB);

	UART2_Put_Char(0x04);
	
	//��һ��������
	UART2_Put_Char(tbuf0[0]);		
	UART2_Put_Char(tbuf0[1]);
	UART2_Put_Char(tbuf0[2]);
	UART2_Put_Char(tbuf0[3]);
	
	UART2_Put_Char(tbuf0[0]^tbuf0[1]^tbuf0[2]^tbuf0[3]^0xaa^0x44^0x04);

}
void UART2_siyuan_accel(uint8_t device_id,
                        float w, float x, float y, float z,
                        float ax, float ay, float az,
                        float gx, float gy, float gz)
{
    uint8_t frame[45]; // 2帧头 + 1ID + 1长度(0x28=40) + 40数据 + 1校验 = 45字节
    uint8_t temp = 0;

    // 帧头
    frame[0] = 0xAA;
    frame[1] = 0x44;
    
    // 设备ID
    frame[2] = device_id;
    
    // 数据长度:  10个float = 40字节 = 0x28
    frame[3] = 0x28;

    // 获取各float数据的字节指针
    uint8_t *pw  = (uint8_t*)&w;
    uint8_t *px  = (uint8_t*)&x;
    uint8_t *py  = (uint8_t*)&y;
    uint8_t *pz  = (uint8_t*)&z;
    uint8_t *pax = (uint8_t*)&ax;
    uint8_t *pay = (uint8_t*)&ay;
    uint8_t *paz = (uint8_t*)&az;
    uint8_t *pgx = (uint8_t*)&gx;
    uint8_t *pgy = (uint8_t*)&gy;
    uint8_t *pgz = (uint8_t*)&gz;

    // 填充数据（小端模式）
    for (int i = 0; i < 4; i++) {
        frame[4  + i] = pw[i];   // 四元数 w
        frame[8  + i] = px[i];   // 四元数 x
        frame[12 + i] = py[i];   // 四元数 y
        frame[16 + i] = pz[i];   // 四元数 z
        frame[20 + i] = pax[i];  // 加速度 ax
        frame[24 + i] = pay[i];  // 加速度 ay
        frame[28 + i] = paz[i];  // 加速度 az
        frame[32 + i] = pgx[i];  // 角速度 gx
        frame[36 + i] = pgy[i];  // 角速度 gy
        frame[40 + i] = pgz[i];  // 角速度 gz
    }

    // 计算校验和（异或校验）
    for (int i = 0; i < 44; i++) {
        temp ^= frame[i];
    }
    frame[44] = temp;

    // 发送数据帧
    for (int i = 0; i < 45; i++) {
        UART2_Put_Char(frame[i]);
    }
}

static uint16_t UART2_Crc16Ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t bit;

    for (i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x8000) != 0)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static uint8_t UART2_TryReadRxByte(uint8_t *value)
{
    uint8_t read_index;

    __disable_irq();
    read_index = s_ucLinkRxReadIndex;
    if (read_index == s_ucLinkRxWriteIndex)
    {
        __enable_irq();
        return 0u;
    }
    *value = s_ucLinkRxBuffer[read_index];
    s_ucLinkRxReadIndex = (uint8_t)((read_index + 1u) & UART2_LINK_RX_BUFFER_MASK);
    __enable_irq();
    return 1u;
}

static void UART2_ResetLinkFrameParser(void)
{
    s_ucLinkFrameIndex = 0u;
    s_ucLinkExpectedFrameSize = 0u;
}

uint8_t UART2_TryReadLinkCommand(UART2_LinkCommand *command)
{
    uint8_t value;

    if (command == 0)
        return 0u;

    while (UART2_TryReadRxByte(&value))
    {
        if ((s_ucLinkFrameIndex == 0u) && (value != 0xA5u))
            continue;

        if ((s_ucLinkFrameIndex == 1u) && (value != 0x5Au))
        {
            s_ucLinkFrameIndex = (value == 0xA5u) ? 1u : 0u;
            continue;
        }

        s_ucLinkFrame[s_ucLinkFrameIndex++] = value;

        if (s_ucLinkFrameIndex == 5u)
        {
            uint8_t payload_length = s_ucLinkFrame[4];
            if ((s_ucLinkFrame[2] != UART2_LINK_PROTOCOL_VERSION) ||
                (payload_length > UART2_LINK_MAX_PAYLOAD))
            {
                s_uiLinkInvalidFrameCount++;
                UART2_ResetLinkFrameParser();
                continue;
            }
            s_ucLinkExpectedFrameSize = (uint8_t)(7u + payload_length);
        }

        if ((s_ucLinkExpectedFrameSize != 0u) &&
            (s_ucLinkFrameIndex == s_ucLinkExpectedFrameSize))
        {
            uint16_t received_crc = (uint16_t)s_ucLinkFrame[s_ucLinkExpectedFrameSize - 2u] |
                                    ((uint16_t)s_ucLinkFrame[s_ucLinkExpectedFrameSize - 1u] << 8);
            uint16_t calculated_crc = UART2_Crc16Ccitt(s_ucLinkFrame,
                                                       (uint16_t)(s_ucLinkExpectedFrameSize - 2u));
            uint8_t link_command = s_ucLinkFrame[3];
            uint8_t payload_length = s_ucLinkFrame[4];

            if (received_crc != calculated_crc)
            {
                s_uiLinkInvalidFrameCount++;
                UART2_ResetLinkFrameParser();
                continue;
            }

            if ((link_command == UART2_LINK_COMMAND_CONFIG_SYNC) && (payload_length == 5u))
            {
                uint8_t transmit_rate_hz = s_ucLinkFrame[5];
                if ((transmit_rate_hz < 1u) || (transmit_rate_hz > 10u))
                {
                    s_uiLinkInvalidFrameCount++;
                    UART2_ResetLinkFrameParser();
                    continue;
                }

                command->Command = link_command;
                command->TransmitRateHz = transmit_rate_hz;
                command->SyncToken = (uint32_t)s_ucLinkFrame[6] |
                                     ((uint32_t)s_ucLinkFrame[7] << 8) |
                                     ((uint32_t)s_ucLinkFrame[8] << 16) |
                                     ((uint32_t)s_ucLinkFrame[9] << 24);
                UART2_ResetLinkFrameParser();
                return 1u;
            }

            if ((link_command == UART2_LINK_COMMAND_PAUSE) && (payload_length == 0u))
            {
                command->Command = link_command;
                command->TransmitRateHz = 0u;
                command->SyncToken = 0u;
                UART2_ResetLinkFrameParser();
                return 1u;
            }

            s_uiLinkInvalidFrameCount++;
            UART2_ResetLinkFrameParser();
        }
    }
    return 0u;
}

uint32_t UART2_GetLinkRxOverflowCount(void)
{
    return s_uiLinkRxOverflowCount;
}

uint32_t UART2_GetLinkInvalidFrameCount(void)
{
    return s_uiLinkInvalidFrameCount;
}

static void UART2_WriteU32Le(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)(value & 0xFF);
    target[1] = (uint8_t)((value >> 8) & 0xFF);
    target[2] = (uint8_t)((value >> 16) & 0xFF);
    target[3] = (uint8_t)((value >> 24) & 0xFF);
}

/*
 * Protocol V2, 36 bytes total:
 * AA 44 | logical id | payload length=30 |
 * quaternion w,x,y,z (16 bytes) | version=2 | flags |
 * source sequence | sender tick ms | stable hardware id | CRC16-CCITT.
 * flags: bit0=metadata, bit1=hardware-derived real-time clock,
 * bit2=main system clock is in the expected 72 MHz range,
 * bit3=latest-only slotted transmission, bit4=Unity schedule synchronized.
 */
void UART2_siyuan_v2(uint8_t device_id,
                     uint32_t hardware_id,
                     uint32_t sequence,
                     uint32_t sender_tick_ms,
                     uint8_t source_flags,
                     float w, float x, float y, float z)
{
    uint8_t frame[36];
    uint8_t *pw = (uint8_t *)&w;
    uint8_t *px = (uint8_t *)&x;
    uint8_t *py = (uint8_t *)&y;
    uint8_t *pz = (uint8_t *)&z;
    uint16_t crc;
    int i;

    frame[0] = 0xAA;
    frame[1] = 0x44;
    frame[2] = device_id;
    frame[3] = 30;

    for (i = 0; i < 4; i++)
    {
        frame[4 + i] = pw[i];
        frame[8 + i] = px[i];
        frame[12 + i] = py[i];
        frame[16 + i] = pz[i];
    }

    frame[20] = 2;
    frame[21] = source_flags;
    UART2_WriteU32Le(&frame[22], sequence);
    UART2_WriteU32Le(&frame[26], sender_tick_ms);
    UART2_WriteU32Le(&frame[30], hardware_id);

    crc = UART2_Crc16Ccitt(frame, 34);
    frame[34] = (uint8_t)(crc & 0xFF);
    frame[35] = (uint8_t)((crc >> 8) & 0xFF);

    for (i = 0; i < 36; i++)
        UART2_Put_Char(frame[i]);
}

//void UART2_siyuan_accel(uint8_t device_id,
//                        float w, float x, float y, float z,
//                        float ax, float ay, float az)
//{
//    uint8_t frame[33]; 
//    uint8_t temp = 0;

//    frame[0] = 0xAA;
//    frame[1] = 0x44;
//    frame[2] = device_id;
//    frame[3] = 0x1C; 

//    uint8_t *pw  = (uint8_t*)&w;
//    uint8_t *px  = (uint8_t*)&x;
//    uint8_t *py  = (uint8_t*)&y;
//    uint8_t *pz  = (uint8_t*)&z;
//    uint8_t *pax = (uint8_t*)&ax;
//    uint8_t *pay = (uint8_t*)&ay;
//    uint8_t *paz = (uint8_t*)&az;

//    for (int i = 0; i < 4; i++) {
//        frame[4  + i] = pw[i];
//        frame[8  + i] = px[i];
//        frame[12 + i] = py[i];
//        frame[16 + i] = pz[i];
//        frame[20 + i] = pax[i];
//        frame[24 + i] = pay[i];
//        frame[28 + i] = paz[i];
//    }

//    for (int i = 0; i < 32; i++) {
//        temp ^= frame[i];
//    }
//    frame[32] = temp;

//    for (int i = 0; i < 33; i++) {
//        UART2_Put_Char(frame[i]);
//    }
//}
void UART2_siyuan(uint8_t device_id, float w, float x, float y, float z) 
{
    uint8_t temp = 0;
    uint8_t frame[21]; // ֡ͷ2 + �豸ID1 + ����1 + ����16 + У���1 = 21�ֽ�

    // ֡ͷ
    frame[0] = 0xAA;
    frame[1] = 0x44;
    
    // �豸ID
    frame[2] = device_id; // ????,??0x01~0x09
    
    // ���ݳ��ȣ��̶�16�ֽڣ�
    frame[3] = 0x10;
    
    // ��Ԫ�����ݣ�С����
    uint8_t *pw = (uint8_t*)&w;
    uint8_t *px = (uint8_t*)&x;
    uint8_t *py = (uint8_t*)&y;
    uint8_t *pz = (uint8_t*)&z;
    
    // �����������w, x, y, z��
    for (int i = 0; i < 4; i++) 
    {
        frame[4 + i]  = pw[i];  // w���ֽڣ��͡��ߣ�
        frame[8 + i]  = px[i];  // x
        frame[12 + i] = py[i];  // y
        frame[16 + i] = pz[i];  // z
    }
    
    // ����У��ͣ�֡ͷ��������ĩβ��
    for (int i = 0; i < 20; i++) 
    {
        temp ^= frame[i];
    }
    frame[20] = temp;
    
    // ��������֡
    for (int i = 0; i < 21; i++) 
    {
        UART2_Put_Char(frame[i]);
    }
}
void UART2_Oula(float num0,float num1,float num2)
{
	unsigned char temp;
	unsigned char tbuf0[4],tbuf1[4],tbuf2[4];
	unsigned char *p = (unsigned char*)&num0 + 3;//ָ��p��ָ��float������ֽ�
	unsigned char *p1 = (unsigned char*)&num1 + 3;//ָ��p��ָ��float������ֽ�
	unsigned char *p2 = (unsigned char*)&num2 + 3;//ָ��p��ָ��float������ֽ�

	//��ȡ��Ӧ��4���ֽڣ��ӵ�λ����λ����ʱ�Ϳ�������hex��ʽ�����ݴ�����
	tbuf0[0] = *(p-3);
	tbuf0[1] = *(p-2);
	tbuf0[2] = *(p-1);
	tbuf0[3] = *p;
	//��ȡ��Ӧ��4���ֽڣ��ӵ�λ����λ����ʱ�Ϳ�������hex��ʽ�����ݴ�����
	tbuf1[0] = *(p1-3);
	tbuf1[1] = *(p1-2);
	tbuf1[2] = *(p1-1);
	tbuf1[3] = *p1;
	//��ȡ��Ӧ��4���ֽڣ��ӵ�λ����λ����ʱ�Ϳ�������hex��ʽ�����ݴ�����
	tbuf2[0] = *(p2-3);
	tbuf2[1] = *(p2-2);
	tbuf2[2] = *(p2-1);
	tbuf2[3] = *p2;
	




	UART2_Put_Char(0xAA);
temp=0xAA;
	UART2_Put_Char(0x44);
temp^=0x44;
	UART2_Put_Char(0x02);
	temp^=0x02;
	UART2_Put_Char(0x0c);
	temp^=0x0c;

	//��һ��������
	UART2_Put_Char(tbuf0[0]);		
	temp^=tbuf0[0];
	UART2_Put_Char(tbuf0[1]);
	temp^=tbuf0[1];
	
	UART2_Put_Char(tbuf0[2]);
	temp^=tbuf0[2];
	UART2_Put_Char(tbuf0[3]);
	temp^=tbuf0[3];
	//�ڶ�����
	UART2_Put_Char(tbuf1[0]);		
	UART2_Put_Char(tbuf1[1]);
	UART2_Put_Char(tbuf1[2]);
	UART2_Put_Char(tbuf1[3]);
	//��������
	UART2_Put_Char(tbuf2[0]);		
	UART2_Put_Char(tbuf2[1]);
	UART2_Put_Char(tbuf2[2]);
	UART2_Put_Char(tbuf2[3]);
	
	
	UART2_Put_Char(temp^tbuf1[0]^tbuf1[1]^tbuf1[2]^tbuf1[3]^tbuf2[0]^tbuf2[1]^tbuf2[2]^tbuf2[3]);


}

void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xAA);
	UART2_Put_Char(0x44);
	UART2_Put_Char(5);
	
	UART2_Put_Char(0x01);
	UART2_Put_Char(0x02);
	UART2_Put_Char(0x03);
	UART2_Put_Char(0x04);
	UART2_Put_Char(0x05);
	UART2_Put_Char(0x01^0x02^0x03^0x04^0x05^0xaa^0x44^0x05);

//	if(yaw<0)yaw=32768-yaw;
//	ctemp=yaw>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=yaw;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	if(pitch<0)pitch=32768-pitch;
//	ctemp=pitch>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=pitch;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	if(roll<0)roll=32768-roll;
//	ctemp=roll>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=roll;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	if(alt<0)alt=32768-alt;
//	ctemp=alt>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=alt;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	if(tempr<0)tempr=32768-tempr;
//	ctemp=tempr>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=tempr;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	if(press<0)press=32768-press;
//	ctemp=press>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=press;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	ctemp=IMUpersec>>8;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;
//	ctemp=IMUpersec;
//	UART2_Put_Char(ctemp);
//	temp+=ctemp;

//	UART2_Put_Char(temp%256);
//	UART2_Put_Char(0xaa);
}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}

void USART2_IRQHandler(void)
{
	uint8_t value;
	uint8_t next_write_index;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		value = (uint8_t)USART_ReceiveData(USART2);
		next_write_index = (uint8_t)((s_ucLinkRxWriteIndex + 1u) & UART2_LINK_RX_BUFFER_MASK);
		if (next_write_index == s_ucLinkRxReadIndex)
		{
			s_uiLinkRxOverflowCount++;
		}
		else
		{
			s_ucLinkRxBuffer[s_ucLinkRxWriteIndex] = value;
			s_ucLinkRxWriteIndex = next_write_index;
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}


//------------------End of File----------------------------
