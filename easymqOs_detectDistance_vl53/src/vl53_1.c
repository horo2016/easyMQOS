#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<stdint.h>

#include<wiringPi.h>
#include "vl53_common.h"
 
#include "VL53_interface.h"
 
#define         VL53_SCL_SET_CH1        {digitalWrite(PIN_SCL, HIGH);}
#define         VL53_SCL_CLR_CH1        {digitalWrite(PIN_SCL, LOW);}

#define        VL53_SDA_SET_CH1        {digitalWrite(PIN_SDA, HIGH);}
#define        VL53_SDA_CLR_CH1        {digitalWrite(PIN_SDA, LOW);}
#define        VL53_SDA_IN_CH1         (digitalRead(PIN_SDA))
#define        VL53_SDA_D_OUT_CH1      {pinMode(PIN_SDA, OUTPUT);}
#define        VL53_SDA_D_IN_CH1       {pinMode(PIN_SDA, INPUT);}





static void VL53_I2c_Start(void)
{
    VL53_SDA_D_OUT_CH1;
    VL53_SDA_SET_CH1;
     VL53_SCL_SET_CH1;
	delayMicroseconds(4);	//是否需要延迟
    VL53_SDA_CLR_CH1;
    delayMicroseconds(4);
     VL53_SCL_CLR_CH1;

}
static void VL53_I2c_Stop(void)
{
    VL53_SDA_D_OUT_CH1;   // 设置SDA为输出方向
	 VL53_SCL_CLR_CH1;  
    VL53_SDA_CLR_CH1;
	delayMicroseconds(4);//是否需要延迟
     VL53_SCL_SET_CH1;
    VL53_SDA_SET_CH1;        // 发送结束信号
    delayMicroseconds(4);
    //VL53_SDA_D_IN_CH1;       //设置SDA为输RU方向
}
static u8 VL_IIC_Wait_Ack(void)
{
	int ucErrTime=0;
	VL53_SDA_D_IN_CH1;  //SDA设置为输入  
	//VL53_SDA_SET_CH1;
    delayMicroseconds(1);	   
	 VL53_SCL_SET_CH1;
    delayMicroseconds(1);	 
	while(VL53_SDA_IN_CH1)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			VL53_I2c_Stop();
            
            printf("time up \n");
			return 1;
		}
	}
	 VL53_SCL_CLR_CH1;//时钟输出0 	   
	return 0;  
}

//产生ACK应答
static void VL_IIC_Ack(void)
{
	 VL53_SCL_CLR_CH1;
	 VL53_SDA_D_OUT_CH1;
	VL53_SDA_CLR_CH1;
	delayMicroseconds(2);
	 VL53_SCL_SET_CH1;
	delayMicroseconds(2);
	 VL53_SCL_CLR_CH1;
}

//不产生ACK应答		    
static void VL_IIC_NAck(void)
{
	 VL53_SCL_CLR_CH1;
	 VL53_SDA_D_OUT_CH1;
	VL53_SDA_SET_CH1;
	delayMicroseconds(2);
	 VL53_SCL_SET_CH1;
	delayMicroseconds(2);
	 VL53_SCL_CLR_CH1;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void VL_IIC_Send_Byte(unsigned char txd)
{                        
    unsigned char t;   
	VL53_SDA_D_OUT_CH1; 	    
     VL53_SCL_CLR_CH1; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7){
            VL53_SDA_SET_CH1;
        }
			
		else{
                VL53_SDA_CLR_CH1;
        }
			
		txd<<=1; 	  
		delayMicroseconds(2);  
		 VL53_SCL_SET_CH1;
		delayMicroseconds(2); 
		 VL53_SCL_CLR_CH1;	
		delayMicroseconds(2);
    }	 
}
static unsigned char VL_IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	VL53_SDA_D_IN_CH1;//
	VL53_SDA_SET_CH1;
	delayMicroseconds(4);;
	for(i=0;i<8;i++ )
	{
		receive<<=1;
		 VL53_SCL_CLR_CH1; 
		delayMicroseconds(4);
		 VL53_SCL_SET_CH1;
		delayMicroseconds(4);
		if(VL53_SDA_IN_CH1)
			receive |= 0x01;   
	  delayMicroseconds(4); //1
	}	
     VL53_SCL_CLR_CH1;	
	return receive;
}


//IIC写一个字节数据
static unsigned char VL_IIC_Write_1Byte(unsigned char SlaveAddress, unsigned char REG_Address,unsigned char REG_data)
{
	VL53_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		VL53_I2c_Stop();//释放总线
		return 1;//没应答则退出

	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();	
	delayMicroseconds(5);
	VL_IIC_Send_Byte(REG_data);
	VL_IIC_Wait_Ack();	
	VL53_I2c_Stop();

	return 0;
}


//IIC读一个字节数据
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	VL53_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack())
	{
		 VL53_I2c_Stop();//释放总线
		 return 1;//没应答则退出
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	 delayMicroseconds(5);
	VL53_I2c_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte();
	VL53_I2c_Stop();

	return 0;
}

//I2C读多个字节数据
static uint8_t VL_I2C_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len)
{
	VL53_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL53_I2c_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	 delayMicroseconds(5);
	VL53_I2c_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	VL_IIC_Wait_Ack();
	while(len)
	{
		*buf = VL_IIC_Read_Byte();
		if(1 == len)
		{
			VL_IIC_NAck();
		}
		else
		{
			VL_IIC_Ack();
		}
		buf++;
		len--;
	}
	VL53_I2c_Stop();

	return 0;
}

//I2C写多个字节数据
static uint8_t VL_I2C_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len)
{
	VL53_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL53_I2c_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);
		VL_IIC_Wait_Ack();
	}
	VL53_I2c_Stop();

	return  0;
}
// 写一个字节
static void VL53_I2c_WrByte(unsigned char dat)
{
    unsigned char i;
    VL53_SDA_D_OUT_CH1;   // 输出
    for(i=0;i!=8;i++)
    {
        if(dat&0x80) {VL53_SDA_SET_CH1;}
        else {VL53_SDA_CLR_CH1;}
        delayMicroseconds(1);;
         VL53_SCL_SET_CH1;
        dat<<=1;
        delayMicroseconds(2);
         VL53_SCL_CLR_CH1;
        delayMicroseconds(1);;
    }
    VL53_SDA_D_IN_CH1;  /*SDA输入方向 */
    VL53_SDA_SET_CH1;
    delayMicroseconds(1);
     VL53_SCL_SET_CH1;  // 等待应答
    delayMicroseconds(2);
     VL53_SCL_CLR_CH1;
    delayMicroseconds(1);
}

 


static int initI2c()
{
    //初始化所用到的IO引脚
    //pinMode(BTN_INTERRUPT, INPUT);
    // INT_EDGE_FALLING：下降沿
    // INT_EDGE_RISING：上升沿
    // INT_EDGE_BOTH: 可上升沿也可以下降沿
    // INT_EDGE_SETUP：保持原有的GPIO初始方式
    //int wiringPiI2CSetup(int devId);
    //wiringPiISR(BTN_INTERRUPT, INT_EDGE_FALLING, &irqHandler);
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    

    digitalWrite(PIN_SCL, HIGH);
    digitalWrite(PIN_SDA, HIGH);
    return 0;
}
static uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}
  int main_1()
{
    u8 data[10];
    u8 i=0;
    u8 beginData = 0;
		u8 val1;
			int ret;
	static first =0;
	if(first ==0){
		first =1;
		//初始化wiringPI的库函数
			if(wiringPiSetup()<0) {
				printf("init wiringPi error\n");
			}
			printf("init wiringPi over\n");
			initI2c();          //spi的初始化
			
		
			ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_IDENTIFICATION_MODEL_ID,&val1);
		if(ret) printf("read failed \n");
			printf("Device ID: \n"); 
			printf("0x%x \n",val1);

			ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_C1_ID,&val1);
		if(ret) printf("read failed \n");

		printf("C1 ID: \n"); 
		printf("0x%x \n",val1);  
			ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_IDENTIFICATION_REVISION_ID,&val1);
		if(ret) printf("read failed \n");

		printf("Revision ID: \n"); 
		printf("0x%x \n",val1);   

		
		
		
			ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,&val1);
		if(ret) printf("read failed \n");
			printf("PRE_RANGE_CONFIG_VCSEL_PERIOD : \n"); 
			printf("%d \n",val1);
			ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,&val1);
			if(ret) printf("read failed \n");
			printf("FINAL_RANGE_CONFIG_VCSEL_PERIOD : \n"); 
			printf("%d \n",val1);
	}
    
 
        VL_IIC_Write_1Byte(address,0x80, 0x01);
        VL_IIC_Write_1Byte(address,0xFF, 0x01);
        VL_IIC_Write_1Byte(address,0x00, 0x00);
        //VL_IIC_Write_1Byte(address,0x91, stop_variable);
        VL_IIC_Write_1Byte(address,0x00, 0x01);
        VL_IIC_Write_1Byte(address,0xFF, 0x00);
        VL_IIC_Write_1Byte(address,0x80, 0x00);


        VL_IIC_Write_1Byte(address, VL53L0X_REG_SYSRANGE_START,0x01);
        int val = 0;
        int cnt = 0;
        while (cnt < 100) { // 1 second waiting time max
            delay(10);
                ret= VL_IIC_Read_1Byte(address,VL53L0X_REG_RESULT_RANGE_STATUS,&val1);
            if(ret) printf("read failed \n");        
            if (val1 & 0x01) break;
            cnt++;
        }
       // if (val1 & 0x01) printf("ready \n"); else printf("not ready \n");
        char gbuf[16];

        VL_I2C_Read_nByte(address, 0x14,gbuf, 12);
        uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
        uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
        uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
        char DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

        //printf("ambient count: %d \n",acnt);
        //printf("signal count:  %d \n",scnt);
        if(dist>20 && dist< 1200)printf("distance  %d \n",dist);
       // printf("status: %d \n ",DeviceRangeStatusInternal);
         
  

    return dist;
}

void set_callback_function(char* _msg ,void(*pfunc)(char*))
{
   (*pfunc)(_msg);
   return ;
}
void vl53_task(void (*func)(char*))
{
	unsigned short buffer[3];
	while(1){

	buffer[0]=	main_1();
	buffer[1]=	main_2();
	buffer[2]=	main_3();
	
   set_callback_function((char *) buffer,func);

	}
}