
#ifndef __SHT75_H__
//12位相对湿度，14位湿度
#define __SHT75_H__
typedef void (*pvoidFun)();
typedef unsigned char (*pucharFun)();
#define SHT75_NUM1      0x01
#define SHT75_NUM2      0x02
#define SHT75_NUM3      0x04

typedef struct SHT75_struct{
		pvoidFun SHT75_data_H;
		pvoidFun SHT75_data_L;
		pucharFun SHT75_read_data;
		pvoidFun SHT75_sck_H;
		pvoidFun SHT75_sck_L;
		float humi;
		float temprature;
		float dew;//露点
}SHT75_Type;

//#define SHT75_VCC_PORT       GPIOF 
//#define SHT75_VCC_PIN         GPIO_Pin_8

#define SHT75_READ_DATA     0x07  //读寄存器
#define SHT75_WRITE_DATA   0x06 //写寄存器
#define SHT75_RESET              0x1E            //复位
#define SHT75_CNVERT_T        0x03     //温度测量
#define SHT75_CNVERT_SH     0x05     //温度测量
void SHT75_translate_start(SHT75_Type *SHT75);
u8 SHT75_run(SHT75_Type *SHT75);
void SHT75_delay_us(u32 count);
void SHT75_delay_ms(u32 count);
extern void SHT75_task_init1(void);
extern void SHT75_task_init2(void);
extern void SHT75_task_init3(void);
float SHT75_get_humi1(void);
float SHT75_get_temp1(void);
float SHT75_get_dew1(void);
float SHT75_get_humi2(void);
float SHT75_get_temp2(void);
float SHT75_get_dew2(void);
float SHT75_get_humi3(void);
float SHT75_get_temp3(void);
float SHT75_get_dew3(void);
u8  SHT75_get_status(void);

#endif
