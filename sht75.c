#ifndef __SHT75_C__
#define __SHT75_C__
#include <rtthread.h>
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm32f10x_rcc.h"
#include"sht75.h"
#include "bme280.h"


#include "myconfig.h"
#include "globalsys.h"
#include "globalval.h"
#ifdef USE_SHT75_TASK


#define SHT75_SCAN_TIM  400
//volatile int is_reseting=0;
//static char reset_flag=0;

//====================================================================
//温湿度控制公共函数
//主要用来和芯片通讯以及温湿度和露点计算
enum 
{
	MEASURE_TEMP,
	MEASURE_SH	
};
#define SHT75_NEED_ACK 1
#define SHT75_NO_ACK 0

#define MAX_ERROR 20

u8  SHT75_status=0x07;  //温湿度传感器故障初始化的时候认为全部都有问题

u8  SHT75_get_status(void)
{
	return SHT75_status;
}

//大约1us  后面替换成systick要做精确的1us
void SHT75_delay_us(u32 count)
{
	//采用timer定时器来延时us不出错
	delay_us(count);
	//将18b20部分进行优化
	//采用OS的延时易出错
	//	rt_hw_us_delay(count+6);
}
float sht_abs(float a)
{
	if(a<0.0001)
		return -a;
	else
		return a;
}
extern void show_tempreture(void);
void SHT75_delay_ms(u32 count)
{
	rt_thread_delay(count);
}

//先写高位
//返回SHT75 确认数据状态
//返回1 数据确认失败
u8 SHT75_write_byte(SHT75_Type *SHT75,u8 data)   
{   
 	u8 i;   
	u8 error;
	SHT75->SHT75_data_L();
	for (i=0x80;i>0;i>>=1)            
	{ 
		if (i & data) 
			SHT75->SHT75_data_H();          
		else 
			SHT75->SHT75_data_L(); 
		SHT75_delay_us(2);
		SHT75->SHT75_sck_H();                          
		SHT75_delay_us(1);	
		SHT75->SHT75_sck_L(); 
		SHT75_delay_us(1);	
		SHT75->SHT75_data_L();
		SHT75_delay_us(1);	
	}
	SHT75->SHT75_data_H();     //释放data 线                      
	SHT75_delay_us(1);	
	error=SHT75->SHT75_read_data();                       

	SHT75->SHT75_sck_H();                             
	SHT75_delay_us(1);	
	SHT75->SHT75_sck_L();       
	SHT75_delay_us(1);
	SHT75->SHT75_data_H();     //释放data 线                      
	SHT75_delay_us(1);	
	return error;                     

}

//先读高位
//读的时候一定要在发送转换命令后延时
u8 SHT75_read_byte(SHT75_Type *SHT75,u8 check_ack)   
{   
 	u8 i;   
	u8 data=0;
	SHT75->SHT75_data_L();
	SHT75_delay_us(1);
	SHT75->SHT75_data_H();
	for (i=0x80;i>0;i/=2)            
	{ 
		if (SHT75->SHT75_read_data()) 
			data=data|i;          
		SHT75->SHT75_sck_H();
		SHT75_delay_us(1);
		SHT75->SHT75_sck_L(); 
		SHT75_delay_us(1);
	}
	if(check_ack)
		SHT75->SHT75_data_L();        
	else
		SHT75->SHT75_data_H();

	SHT75_delay_us(1);
	SHT75->SHT75_sck_H();                             
	SHT75_delay_us(3);	
	SHT75->SHT75_sck_L();       
	SHT75_delay_us(1);
	SHT75->SHT75_data_H();
	return data;                     

}

//sht 复位信号
void SHT75_reset_signal(SHT75_Type *SHT75)
{
  	u8 i; 
  	SHT75->SHT75_data_H(); 
	SHT75->SHT75_sck_L();       
  	for(i=0;i<9;i++)                  //9 SCK cycles
  	{ 
		SHT75->SHT75_sck_H();       
		SHT75_delay_us(1);	
		SHT75->SHT75_sck_L();       
  	}
  	SHT75_translate_start(SHT75);                   //transmission start
}

//通过命令让芯片复位
u8 SHT75_soft_reset(SHT75_Type *SHT75)
{
	u8 error;
	SHT75_reset_signal(SHT75);
	error=SHT75_write_byte(SHT75,SHT75_RESET);
	return error;
	
}
//传输命令头
void SHT75_translate_start(SHT75_Type *SHT75)
{

	SHT75->SHT75_data_H();    //释放数据线
	SHT75_delay_us(1);	

	SHT75->SHT75_sck_L();       //拉低时钟
	SHT75_delay_us(1);	
	SHT75->SHT75_sck_H();      //拉高时钟                       

	SHT75_delay_us(1);	
	SHT75->SHT75_data_L();      //拉低数据线
	SHT75_delay_us(2);	

	SHT75->SHT75_sck_L();       //拉低时钟
	SHT75_delay_us(1);	
	SHT75->SHT75_sck_H();        //拉高时钟                     

	SHT75_delay_us(1);	
	SHT75->SHT75_data_H();   //释放数据线
	SHT75_delay_us(1);	

	SHT75->SHT75_sck_L();       //拉低时钟
	SHT75_delay_us(1);	

}

u8 SHT75_read_status_reg(SHT75_Type *SHT75,u8 *p_value, u8 *p_checksum)
{ 
  	u8 error=0;
  	SHT75_translate_start(SHT75);                   
  	error=SHT75_write_byte(SHT75,SHT75_READ_DATA); 
	SHT75_delay_ms(200);//延时200ms
  	*p_value=SHT75_read_byte(SHT75,SHT75_NEED_ACK);        
  	*p_checksum=SHT75_read_byte(SHT75,SHT75_NO_ACK);     
  	return error;                     
}
static	uint8_t	const CRC8Table[256] =
	{
		0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
		67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109, 
		134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168, 
		197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235, 
		61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19, 
		126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80, 
		187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149, 
		248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
		122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84, 
		57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23, 
		252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,  
		191, 142, 221, 236,123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145, 
		71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105, 
		4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42, 
		193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239, 
		130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172, 
	};

//测量
//p_value 存储测量值 2个字节
//p_checksum存储CRC校验
//mode  温度还是湿度
u8 SHT75_measure(SHT75_Type *SHT75,u8 *p_value, u8 *p_checksum, u8 mode)
{ 
  	u32 i=10000;
	u8 error=0;
	u8	SensorCRC = 0u;
	u8 CRC8=0;
  	SHT75_translate_start(SHT75);                   //transmission start
  	switch(mode)
	{                     
    		case MEASURE_TEMP: 
			error=SHT75_write_byte(SHT75,SHT75_CNVERT_T);
			if(error==1)
			{
				while(i--);
				SHT75_reset_signal(SHT75);
				error=SHT75_write_byte(SHT75,SHT75_CNVERT_T);
			}
			break;
    		case MEASURE_SH: 
			error=SHT75_write_byte(SHT75,SHT75_CNVERT_SH); 
			if(error==1)
			{
				while(i--);
				SHT75_reset_signal(SHT75);
				error=SHT75_write_byte(SHT75,SHT75_CNVERT_SH);
			}
			break;
    		default :
			break;	 
  	}
	if(error)//传感器错误
		return error;
	SHT75_delay_ms(800);
  	* (p_value+1)=SHT75_read_byte(SHT75,SHT75_NEED_ACK);    
  	*(p_value) =SHT75_read_byte(SHT75,SHT75_NEED_ACK); 
  	*p_checksum =SHT75_read_byte(SHT75,SHT75_NO_ACK);

	SensorCRC = *p_checksum;
	SensorCRC = (( SensorCRC & 0x0Fu ) << 4 ) | (( SensorCRC & 0xF0u ) >> 4 );
	SensorCRC = (( SensorCRC & 0x33u ) << 2 ) | (( SensorCRC & 0xCCu ) >> 2 );
	SensorCRC = (( SensorCRC & 0x55u ) << 1 ) | (( SensorCRC & 0xAAu ) >> 1 );

	CRC8 = 0;
	CRC8 = CRC8Table[CRC8 ^ (mode ?  0x05:0x03 )];
	CRC8 = CRC8Table[CRC8 ^ (* (p_value+1))];
	CRC8 = CRC8Table[CRC8 ^ (* (p_value))];
	CRC8 = CRC8Table[CRC8 ^ SensorCRC];
	
	if ( 0u != CRC8 )
		error=20;	
  	return error;
}

void SHT75_calc(float *p_humidity ,float *p_temperature)
{ 
	const float C1=-2.0468;              
	const float C2=+0.0367;           
	const float C3=-1.5955;     //运算的时候再除以1000 000   
	const float T1=+0.01;             
	const float T2=+0.00008;           	

	float rh=*p_humidity;             
	float t=*p_temperature;           
	float rh_lin;                    
	float rh_true;                    
	float t_C;                       

	t_C=t*0.01 - 39.7;                 
	rh_lin=C3*rh/1000*rh/1000 + C2*rh + C1;     
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   
	if(rh_true>100)
		rh_true=100;      
	if(rh_true<0.1)
		rh_true=0.1;      

	*p_temperature=t_C;               
	*p_humidity=rh_true;  
}

float SHT15_Calculate_Dew_Point(float Temp,float Rh)
{
	const float t1=243.12;
	const float t2=272.62;
	const float m1=17.62;
	const float m2=22.46;
	float Dew=0.0;
	float a=0,b=0;
	float c=0,d=0;
	if(Temp>0&&Temp<50)
	{
		a=log(Rh*0.01)+m1*Temp/(t1+Temp);
		b=m1-log(Rh*0.01)-m1*Temp/(t1+Temp);
		Dew=(u16)t1*a/b;
	}
	if(Temp>-40&&Temp<0)
	{
		c=log(Rh*0.01)+m2*Temp/(t2+Temp);
		d=m2-log(Rh*0.01)-m2*Temp/(t2+Temp);
		Dew=t2*c/d;
	}
	return Dew;
}
u8 SHT75_run(SHT75_Type *SHT75)
{
	u8 error,checksum;
	u16 temp_temp2=0;
	u16 temp_sh=0;
	float humi=0.0;
	float temp=0.0;
	float dew_point=0.0;
	error=0;
	error+=SHT75_measure(SHT75,(u8*) &temp_temp2,&checksum,MEASURE_TEMP);  //测量温度
 	error+=SHT75_measure(SHT75,(u8*) &temp_sh,&checksum,MEASURE_SH);  //测量湿度
	if(error!=0)   //传感器错误 需要记录哪只传感器有问题
	{
		SHT75_reset_signal(SHT75); 
	}
	else
	{
		humi= (float)temp_sh;
		temp= (float)temp_temp2;

		SHT75_calc(&humi,&temp); 
///		为避免大量的浮点运算,在不需要露点的时候不计算露点
///		dew_point=SHT15_Calculate_Dew_Point(temp,humi); //计算露点
	}
	SHT75->temprature=temp;
	SHT75->humi=humi;
	SHT75->dew=dew_point;
	return error;

}

void SHT75_reset(void)
{
	#define SHT75_RESET_TIM	3000
	{
		Set_SHT75_Disable();
		rt_thread_delay(SHT75_RESET_TIM);
		Set_SHT75_Enable();
		rt_thread_delay(SHT75_RESET_TIM);
	}

}
//===================================================================
#ifdef USE_HUMI1
SHT75_Type  sht75_1;
void SHT75_data_H1(void) 
{
	GPIO_SetBits(SHT75_DATA_PORT,SHT75_DATA_PIN);
}
void  SHT75_data_L1(void)
{
	GPIO_ResetBits(SHT75_DATA_PORT,SHT75_DATA_PIN);
}
unsigned char SHT75_read_data1(void)
{
	return GPIO_ReadInputDataBit(SHT75_DATA_PORT,SHT75_DATA_PIN) ;
}
float bak_temp=-80,bak_humi=-80;
void SHT75_sck_H1(void)
{
	GPIO_SetBits(SHT75_SCK_PORT,SHT75_SCK_PIN);
}
void  SHT75_sck_L1(void) 
{
	GPIO_ResetBits(SHT75_SCK_PORT,SHT75_SCK_PIN);
}
//SHT75  IO 配置
void SHT75_init1(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   
    
 	RCC_APB2PeriphClockCmd(SHT75_RCC_PORT|SHT75_EN_RCC, ENABLE);   
  
 	GPIO_InitStructure.GPIO_Pin = SHT75_DATA_PIN;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_DATA_PORT, &GPIO_InitStructure);   

	 GPIO_InitStructure.GPIO_Pin = SHT75_SCK_PIN;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_SCK_PORT, &GPIO_InitStructure);


	 GPIO_InitStructure.GPIO_Pin = SHT75_EN_PIN;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_EN_PORT, &GPIO_InitStructure);

	//sht75 power supply enable
	//Set_SHT75_Enable();
	//rt_thread_delay(1500);

	SHT75_reset();
	
//	TIM6_NVIC_Configuration();

	sht75_1.dew=0;
	sht75_1.humi=0;
	sht75_1.dew=0;
	sht75_1.SHT75_data_H=SHT75_data_H1;
	sht75_1.SHT75_data_L=SHT75_data_L1;
	sht75_1.SHT75_read_data=SHT75_read_data1;
	sht75_1.SHT75_sck_H=SHT75_sck_H1;
	sht75_1.SHT75_sck_L=SHT75_sck_L1;
	
	bak_temp=-80;
	bak_humi=-80;
}
//温湿度
void SHT75_task_entry_1(void *param)
{
	int count =0;
	u8 error;
	int temp_k=0;
	int reset_count=0;   //自动重启
	SHT75_init1();
	while(1)
	{
		if(is_reseting==0)
		{
			error=SHT75_run(&sht75_1);
			if(error)
				SHT75_status|=SHT75_NUM1;
			else
				SHT75_status&=~SHT75_NUM1;

//传感器通讯故障后,温湿度值采用之前的
			if(error)  //出错
			{
				count++;
				if(count>MAX_ERROR)
				{
					count=0;
					reset_flag =1;
//					modifyro(OUTSIDE_HUM,0,0);
//					modifyro(OUTSIDE_TMP,0,0);
					//温湿度0.5范围波动
					if(bak_temp>-50)
						modifyro(OUTSIDE_TMP,(int)(bak_temp*1000)+(getro(CURSIGN)&0x1ff),0);
					if(bak_humi>0)
						modifyro(OUTSIDE_HUM,(int)(bak_humi*1000)+(getro(CURSIGN)&0x1ff),0);
					if(error!=20)//crc错误暂不 报传感器故障
						Set_Sht75_1_StaFlag();
					rt_kprintf("通讯错误.\n") ;
				}
			}
			else 
			{
				reset_count++;
				if(reset_count>2000)//大概5000*0.4S重启一次
				{
					reset_flag=1;
					reset_count=0;
				}
				//湿度低于1%或者温度大于60度认为异常
				if(SHT75_get_humi1()<0.5 ||SHT75_get_temp1()>80 )// 1%
				{
					
					rt_kprintf("湿度过低 %d %d\n",(int)(SHT75_get_humi1()*100),(int)(SHT75_get_temp1()*100));
					reset_flag=1;
				}
				else  //温湿度正常才赋值
				{	
					if(bak_humi<=0||sht_abs(SHT75_get_humi1()-bak_humi)<15)
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)&0xffff);
						temp_k+=(int)(SHT75_get_humi1()*1000);
						if(temp_k<0)
							temp_k=0;
						else if(temp_k>100000)
							temp_k=100000;
						modifyro(OUTSIDE_HUM,temp_k,0);
						bak_humi= SHT75_get_humi1();
						Clr_Sht75_1_StaFlag();
					}
					else if(bak_humi>0)
					{
						reset_flag=1;
						
						rt_kprintf("湿度差值错误%d %d\n",(int)(bak_humi*100),(int)(SHT75_get_humi1()*100));
					}
					if(bak_temp<=-50||sht_abs(SHT75_get_temp1()-bak_temp<10))
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)>>16);
						modifyro(OUTSIDE_TMP,(int)(SHT75_get_temp1()*1000)+temp_k,0);
						bak_temp= SHT75_get_temp1();
						Clr_Sht75_1_StaFlag();
					}
					else if(bak_temp>-50)
					{
						reset_flag=1;
						rt_kprintf("温度差值错误 %d %d\n",(int)(bak_temp*100),(int)(SHT75_get_temp1()*100));
					}
				}
			}
		}
		rt_thread_delay(SHT75_SCAN_TIM);
	}
}


void SHT75_reset_task_entry_1(void *param)
{
	#define SHT75_RESET_SCAN_TIM 100
	while(1)
	{
		if(reset_flag)
		{
			reset_flag =0;
			is_reseting=1;

#ifdef USE_B_2GATE
			if(1)//getrw(HUMI_CC)==2)
#else
			if(getrw(HUMI_CC)==0)
#endif
				SHT75_reset();
			else if(getrw(HUMI_CC)==1)
				BME280_reset();
			is_reseting=0;
//			bak_temp=-80;  //复位后备份值还是采用之前的
//			bak_humi=-80;
		}	
		rt_thread_delay(SHT75_RESET_SCAN_TIM);
	}
}

void SHT75_task_init1()
{
	rt_thread_t thread;//thread_reset;
	
    /* create led1 thread */
    thread = rt_thread_create("SHT751",
                              SHT75_task_entry_1, RT_NULL,
                              1024,
                              THREAD_SHT75_TASK1_PRIORITY, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /* create led1 thread */
    thread= rt_thread_create("sht_re",
                              SHT75_reset_task_entry_1, RT_NULL,
                             256,
                              THREAD_SHT75_RESET_PRIORITY, 10);

	
    if (thread != RT_NULL)
        rt_thread_startup(thread);

//	    if (thread_reset != RT_NULL)
 //       rt_thread_startup(thread_reset);

}
float SHT75_get_humi1(void)
{
	return sht75_1.humi;
}
float SHT75_get_temp1(void)
{
	return sht75_1.temprature;
}
float SHT75_get_dew1(void)
{
	return sht75_1.dew;
}
#endif

#ifdef USE_HUMI2
SHT75_Type  sht75_2;


void SHT75_data_H2(void) 
{
	GPIO_SetBits(SHT75_DATA_PORT2,SHT75_DATA_PIN2);
}
void  SHT75_data_L2(void)
{
	GPIO_ResetBits(SHT75_DATA_PORT2,SHT75_DATA_PIN2);
}
unsigned char SHT75_read_data2(void)
{
	return GPIO_ReadInputDataBit(SHT75_DATA_PORT2,SHT75_DATA_PIN2) ;
}

void SHT75_sck_H2(void)
{
	GPIO_SetBits(SHT75_SCK_PORT2,SHT75_SCK_PIN2);
}
void  SHT75_sck_L2(void) 
{
	GPIO_ResetBits(SHT75_SCK_PORT2,SHT75_SCK_PIN2);
}
//SHT75  IO 配置
void SHT75_init2(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   
    
 	RCC_APB2PeriphClockCmd(SHT75_RCC_PORT2, ENABLE);   
  
 	GPIO_InitStructure.GPIO_Pin = SHT75_DATA_PIN2;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_DATA_PORT2, &GPIO_InitStructure); 

	 GPIO_InitStructure.GPIO_Pin = SHT75_SCK_PIN2;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_SCK_PORT2, &GPIO_InitStructure);   
	//TIM6_NVIC_Configuration();

		 GPIO_InitStructure.GPIO_Pin = SHT75_EN_PIN;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_EN_PORT, &GPIO_InitStructure);

	//sht75 power supply enable
	//Set_SHT75_Enable();
	//rt_thread_delay(1500);

	SHT75_reset();
	sht75_2.dew=0;
	sht75_2.humi=0;
	sht75_2.dew=0;
	sht75_2.SHT75_data_H=SHT75_data_H2;
	sht75_2.SHT75_data_L=SHT75_data_L2;
	sht75_2.SHT75_read_data=SHT75_read_data2;
	sht75_2.SHT75_sck_H=SHT75_sck_H2;
	sht75_2.SHT75_sck_L=SHT75_sck_L2;
}
//温湿度
void SHT75_task_entry_2(void *param)
{
	int count =0;
	u8 error;
	SHT75_init2();
	while(1)
	{
		if(is_reseting==0)
		{
			error=SHT75_run(&sht75_2);
			if(error)
				SHT75_status|=SHT75_NUM2;
			else
				SHT75_status&=~SHT75_NUM2;
			if(error)  //出错
			{
				count++;
	//			rt_kprintf("%d   \n",count);
				if(count>MAX_ERROR)
				{
	//				rt_kprintf("HUN2 RESET !\n");
					count=0;
					reset_flag=1;
					modifyro(UNDER_COVER_HUM,0,0);
					modifyro(UNDER_COVER_TMP,0,0);
					Set_Sht75_2_StaFlag();
				}
			}
			else 
			{
	//			rt_kprintf("%d   \n",count);
				count--;
				if(count<0)
					count=0;
				if(SHT75_get_humi2()<5 ||SHT75_get_temp2()>80 )// 1%
				{
					reset_flag=1;
				}
				else  
				{	
					Clr_Sht75_2_StaFlag();
					modifyro(UNDER_COVER_HUM,(int)(SHT75_get_humi2()*1000),0);
					modifyro(UNDER_COVER_TMP,(int)(SHT75_get_temp2()*1000),0);
				}
			}
		}
//		rt_kprintf(">>hum2 is %d\r\n",(int)(SHT75_get_humi2()*1000));
//		rt_kprintf(">>tmp2 is %d\r\n",(int)(SHT75_get_temp2()*1000));
		rt_thread_delay(SHT75_SCAN_TIM);
	}
}

void SHT75_task_init2()
{
	rt_thread_t thread;
	
    /* create led1 thread */
    thread = rt_thread_create("SHT752",
                              SHT75_task_entry_2, RT_NULL,
                              1024,
                              THREAD_SHT75_TASK2_PRIORITY, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

}
float SHT75_get_humi2(void)
{
	return (sht75_2.humi);
}
float SHT75_get_temp2(void)
{
	return (sht75_2.temprature);
}
float SHT75_get_dew2(void)
{
	return (sht75_2.dew);
}
#endif

#ifdef USE_HUMI3
SHT75_Type  sht75_3;
void SHT75_data_H3(void) 
{
	GPIO_SetBits(SHT75_DATA_PORT3,SHT75_DATA_PIN3);
}
void  SHT75_data_L3(void)
{
	GPIO_ResetBits(SHT75_DATA_PORT3,SHT75_DATA_PIN3);
}
unsigned char SHT75_read_data3(void)
{
	return GPIO_ReadInputDataBit(SHT75_DATA_PORT3,SHT75_DATA_PIN3) ;
}

void SHT75_sck_H3(void)
{
	GPIO_SetBits(SHT75_SCK_PORT3,SHT75_SCK_PIN3);
}
void  SHT75_sck_L3(void) 
{
	GPIO_ResetBits(SHT75_SCK_PORT3,SHT75_SCK_PIN3);
}
void SHT75_init3(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   
    
 	RCC_APB2PeriphClockCmd(SHT75_RCC_PORT3, ENABLE);   
  
 	GPIO_InitStructure.GPIO_Pin = SHT75_DATA_PIN3;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_DATA_PORT3, &GPIO_InitStructure); 

	 GPIO_InitStructure.GPIO_Pin = SHT75_SCK_PIN3;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_SCK_PORT3 ,&GPIO_InitStructure); 

		 GPIO_InitStructure.GPIO_Pin = SHT75_EN_PIN;   
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //开漏输出   
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //22M时钟速度   
 	GPIO_Init(SHT75_EN_PORT, &GPIO_InitStructure);

	//sht75 power supply enable
	//Set_SHT75_Enable();
	//rt_thread_delay(1500);

	SHT75_reset();

	sht75_3.dew=0;
	sht75_3.humi=0;
	sht75_3.dew=0;
	sht75_3.SHT75_data_H=SHT75_data_H3;
	sht75_3.SHT75_data_L=SHT75_data_L3;
	sht75_3.SHT75_read_data=SHT75_read_data3;
	sht75_3.SHT75_sck_H=SHT75_sck_H3;
	sht75_3.SHT75_sck_L=SHT75_sck_L3;

}
//温湿度
void SHT75_task_entry_3(void *param)
{
	int count =0;
	u8 error;   
	SHT75_init3();
	while(1)
	{
		if(is_reseting==0)
		{
			error=SHT75_run(&sht75_3);
			if(error)
			SHT75_status|=SHT75_NUM3;
			else
				SHT75_status&=~SHT75_NUM3;

			if(error)  //出错
			{
				count++;
				if(count>MAX_ERROR)
				{
					count=0;
					SHT75_reset();
					//reset_flag=1;
					SHT75_init3();

					modifyro(UNDER_COVER_HUM,0,0);
					modifyro(UNDER_COVER_TMP,0,0);
					Set_Sht75_3_StaFlag();
				}
			}
			else 
			{
				Clr_Sht75_3_StaFlag();
				count=0;
				//湿度低于1%或者温度大于60度认为异常
				if(SHT75_get_humi3()<5 ||SHT75_get_temp3()>80 )// 1%
				{
					SHT75_reset();
					SHT75_init3();
				}
				else  //温湿度正常才赋值
				{	
					
					modifyro(UNDER_COVER_HUM,(int)(SHT75_get_humi2()*1000),0);
					modifyro(UNDER_COVER_TMP,(int)(SHT75_get_temp2()*1000),0);
				}
			}
		}
		
		rt_thread_delay(SHT75_SCAN_TIM);
	}
}

void SHT75_task_init3()
{
	rt_thread_t thread;
	
    /* create led1 thread */
    thread = rt_thread_create("SHT753",
                              SHT75_task_entry_3, RT_NULL,
                              512,
                              THREAD_SHT75_TASK3_PRIORITY, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

}
float SHT75_get_humi3(void)
{
	return sht75_3.humi;
}
float SHT75_get_temp3(void)
{
	return sht75_3.temprature;
}
float SHT75_get_dew3(void)
{
	return sht75_3.dew;
}

#endif
#endif

//环境温湿度SHT75和BME280   根据屏设置选择
void humi_task_select(void *param)
{
	int count =0;
	u8 error;
	int reset_count=0;   //自动重启
	int bak_select=0;        //默认选择sht75
	int Bme280_state=0;
	int bak_Bme280_state=0;
	int temp_k=0;
	uint8_t buf_Bme280[25]={0};
	BME280_init1();
	SHT75_init1();
//扬尘 0  sht75 1 bme280  2 485的
//5005     0 485     1 bme280  2 sht
	while(1)
	{
		if(bak_select!=getrw(HUMI_CC))
		{
#ifdef USE_B_2GATE
			if(getrw(HUMI_CC)==2)//sht75
#else
			if(getrw(HUMI_CC)==0)
#endif
			{
				SHT75_init1();
				rt_kprintf("clean 00\n");
			}
			else if(getrw(HUMI_CC)==1)
			{
				BME280_init1();
				rt_kprintf("clean 01\n");
			}
			else
			{
				Clr_Sht75_1_StaFlag();
				rt_kprintf("clean 02\n");
			}
			bak_select=getrw(HUMI_CC);
		}
		
#ifdef USE_B_2GATE
		if(is_reseting==0 && getrw(HUMI_CC)==2)
#else
		if(is_reseting==0 && getrw(HUMI_CC)==0)
#endif
		{
			error=SHT75_run(&sht75_1);
			if(error)
				SHT75_status|=SHT75_NUM1;
			else
				SHT75_status&=~SHT75_NUM1;

//传感器通讯故障后,温湿度值采用之前的
			if(error)  //出错
			{
				count++;
				if(count>MAX_ERROR)
				{
					count=0;
					reset_flag =1;
//					modifyro(OUTSIDE_HUM,0,0);
//					modifyro(OUTSIDE_TMP,0,0);
					//温湿度0.5范围波动
					if(bak_temp>-50)
						modifyro(OUTSIDE_TMP,(int)(bak_temp*1000)+(getro(CURSIGN)&0x1ff),0);
					if(bak_humi>0)
						modifyro(OUTSIDE_HUM,(int)(bak_humi*1000)+(getro(CURSIGN)&0x1ff),0);
					if(error!=20)//crc错误暂不 报传感器故障
						Set_Sht75_1_StaFlag();
					rt_kprintf("通讯错误.\n") ;
				}
			}
			else 
			{
				reset_count++;
				if(reset_count>2000)//大概5000*0.4S重启一次
				{
					reset_flag=1;
					reset_count=0;
				}
				//湿度低于1%或者温度大于60度认为异常
				if(SHT75_get_humi1()<0.5 ||SHT75_get_temp1()>80 )// 1%
				{
					
					rt_kprintf("湿度过低 %d %d\n",(int)(SHT75_get_humi1()*100),(int)(SHT75_get_temp1()*100));
					reset_flag=1;
				}
				else  //温湿度正常才赋值
				{	
					if(bak_humi<=0||sht_abs(SHT75_get_humi1()-bak_humi)<15)
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)&0xffff);
						temp_k+=(int)(SHT75_get_humi1()*1000);
						if(temp_k<0)
							temp_k=0;
						else if(temp_k>100000)
							temp_k=100000;
						modifyro(OUTSIDE_HUM,temp_k,0);
						bak_humi= SHT75_get_humi1();
						Clr_Sht75_1_StaFlag();
					}
					else if(bak_humi>0)
					{
						reset_flag=1;
						
						rt_kprintf("湿度差值错误%d %d\n",(int)(bak_humi*100),(int)(SHT75_get_humi1()*100));
					}
					if(bak_temp<=-50||sht_abs(SHT75_get_temp1()-bak_temp<10))
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)>>16);
						modifyro(OUTSIDE_TMP,(int)(SHT75_get_temp1()*1000)+temp_k,0);
						bak_temp= SHT75_get_temp1();
						Clr_Sht75_1_StaFlag();
					}
					else if(bak_temp>-50)
					{
						reset_flag=1;
						rt_kprintf("温度差值错误 %d %d\n",(int)(bak_temp*100),(int)(SHT75_get_temp1()*100));
					}
				}
			}
		}
		if(is_reseting==0 && getrw(HUMI_CC)==1)
		{
			Bme280_state=Get_Bme280Value(&BME280_Sensor1);
			if(Bme280_state)
			{
				if(bak_Bme280_state==0)
				{
					Bme280_InitParam(&BME280_Sensor1);
				}
				else
				{
					/* 读取温度压力补偿值 */
					IIC_Read_NBytes(BME280_Sensor1.I2C, BME280_DEVICE_ADDR, 0x88 , 24, buf_Bme280);
					BME280_Sensor1.Register.dig_T1 = buf_Bme280[1] << 8 | buf_Bme280[0];
					BME280_Sensor1.Register.dig_T2 = buf_Bme280[3] << 8 | buf_Bme280[2];
					BME280_Sensor1.Register.dig_T3 = buf_Bme280[5] << 8 | buf_Bme280[4];
				}
//				count=0;
				if(bak_bme280_temp<-50||bme280_abs(bak_bme280_temp-BME280_Sensor1.Temperature)<10000)
				{
					if(BME280_Sensor1.Temperature>-50000 &&BME280_Sensor1.Temperature<70000)
					{							
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)>>16);
						modifyro(OUTSIDE_TMP,BME280_Sensor1.Temperature+temp_k,0);
						count=0;
//						modifyro(OUTSIDE_TMP,BME280_Sensor1.Temperature,0);
						bak_bme280_temp=BME280_Sensor1.Temperature;
					}
					else
					{
						count++;	
					}
					
				}
				else
				{
					count++;
				}
				if(bak_bme280_humi<0||bme280_abs(bak_bme280_humi-BME280_Sensor1.Humidity)<15000)
				{
					if(BME280_Sensor1.Humidity>=0 &&BME280_Sensor1.Humidity<=100000)
					{
						temp_k=(short)(getrw(HJ_TEMP_HUMI_K)&0xffff);
						temp_k+=BME280_Sensor1.Humidity;
						if(temp_k<0)
							temp_k=0;
						else if(temp_k>100000)
							temp_k=100000;
						modifyro(OUTSIDE_HUM,temp_k,0);
						bak_bme280_humi=BME280_Sensor1.Humidity;
					}
				}
				else
					count++;
				if(count>MAX_BME280_ERROR)
				{
					bak_bme280_temp=-80;
					bak_bme280_humi=-80;
					count=0;
				}
	//			modifyro(OUTSIDE_HUM,BME280_Sensor1.Humidity,0);
	//			rt_kprintf("Tempe1=%d\n",BME280_Sensor1.Temperature);
				Clr_Sht75_1_StaFlag();
			}
			else
			{
				count++;
				if(count>MAX_BME280_ERROR)
				{
					count=0;
					modifyro(OUTSIDE_TMP,0,0);
					modifyro(OUTSIDE_HUM,0,0);
					Set_Sht75_1_StaFlag();
	
					rt_kprintf("set 11\n");
				}
			}
			bak_Bme280_state=Bme280_state;
		}
		rt_thread_delay(SHT75_SCAN_TIM);
	}
}


void humi_task_select_init()
{
	rt_thread_t thread;
	
    /* create led1 thread */
    thread = rt_thread_create("humi",
                              humi_task_select, RT_NULL,
                              1536,
                              THREAD_SHT75_TASK3_PRIORITY, 50);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    /* create led1 thread */
    thread= rt_thread_create("sht_re",
                              SHT75_reset_task_entry_1, RT_NULL,
                             256,
                              THREAD_SHT75_RESET_PRIORITY, 10);

	
    if (thread != RT_NULL)
        rt_thread_startup(thread);

}

#endif                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
