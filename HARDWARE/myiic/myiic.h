#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "delay.h"

//IO方向设置
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

//IO操作函数
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

typedef struct {
    float rol;
    float pit;
    float yaw;
} T_float_angle;
typedef struct {
    float X;
    float Y;
    float Z;
} T_float_xyz;
typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} T_int16_xyz;
extern T_int16_xyz 	Acc,Gyr;
extern T_int16_xyz		Acc_AVG;
extern T_float_angle 	Att_Angle;
extern u8 Count_Flag;

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节

void WriteData(u8 DevID,u8 Addr,u8 Dat);
void ReadData(u8 DevID,u8 Addr,u8 *Pbuf,u8 Num);


u8 Prepare_Data(T_int16_xyz *acc_in,T_int16_xyz *acc_out);
void IMUupdate(T_int16_xyz *gyr, T_int16_xyz *acc, T_float_angle *angle);
#endif
