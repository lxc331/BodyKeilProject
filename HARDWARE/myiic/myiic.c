#include "myiic.h"
#include "math.h"
#define RtA 		57.295779f
#define AtR    		0.0174533f
#define Acc_G 		0.0095605f
#define Gyro_G 		0.0305f
#define Gyro_Gr		0.0010653f
#define FILTER_NUM 	20

T_int16_xyz 	Acc,Gyr;   //从mpu6050读取到的三轴加速度和三轴陀螺数据
T_int16_xyz		Acc_AVG;		//
u8 Count_Flag;
T_float_angle 	Att_Angle;//转换出来的欧拉角

u8 Prepare_Data(T_int16_xyz *acc_in,T_int16_xyz *acc_out)
{
    static uint8_t 	filter_cnt=0;
    static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
    int32_t temp1=0,temp2=0,temp3=0;
    uint8_t i;

    ACC_X_BUF[filter_cnt] = acc_in->X;
    ACC_Y_BUF[filter_cnt] = acc_in->Y;
    ACC_Z_BUF[filter_cnt] = acc_in->Z;

    for(i=0; i<FILTER_NUM; i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];
    }
    acc_out->X = temp1 / FILTER_NUM;
    acc_out->Y = temp2 / FILTER_NUM;
    acc_out->Z = temp3 / FILTER_NUM;
    filter_cnt++;
    if(filter_cnt==FILTER_NUM)
    {
        filter_cnt=0;
        return 1;
    }
    else return 0;
}



//初始化IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	//PB6,PB7 输出高
}
//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA=1;
    IIC_SCL=1;
    delay_us(4);
    IIC_SDA=0;//START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT();//sda线输出
    IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
    IIC_SCL=1;
    delay_us(4);
    IIC_SDA=1;//发送I2C总线结束信号
    delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN();      //SDA设置为输入
    IIC_SDA=1;
    delay_us(1);
    IIC_SCL=1;
    delay_us(1);
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL=0;//时钟输出0
    return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=0;
    delay_us(2);
    IIC_SCL=1;
    delay_us(2);
    IIC_SCL=0;
}
//不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=1;
    delay_us(2);
    IIC_SCL=1;
    delay_us(2);
    IIC_SCL=0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0; t<8; t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
        delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_SCL=1;
        delay_us(2);
        IIC_SCL=0;
        delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0; i<8; i++ )
    {
        IIC_SCL=0;
        delay_us(2);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}
void WriteData(u8 DevID,u8 Addr,u8 Dat)
{
    IIC_Start();
    IIC_Send_Byte(DevID << 1| 0);	//发送设备地址和写信号
    IIC_Wait_Ack();
    IIC_Send_Byte(Addr);
    IIC_Wait_Ack();
    IIC_Send_Byte(Dat);
    IIC_Wait_Ack();
    IIC_Stop();
    delay_ms(10);
}
void ReadData(u8 DevID,u8 Addr,u8 *Pbuf,u8 Num)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte(DevID << 1 | 0);	//发送设备地址和写信号
    IIC_Wait_Ack();
    IIC_Send_Byte(Addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(DevID << 1 | 1);	//发送设备地址和读信号
    IIC_Wait_Ack();
    for(i = 0; i < (Num - 1); i ++)
    {
        Pbuf[i] = IIC_Read_Byte(1);
    }
    Pbuf[i] = IIC_Read_Byte(0);
    IIC_Stop();
//	delay_ms(5);
}

