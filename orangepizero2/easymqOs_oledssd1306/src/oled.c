#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<stdint.h>

#include<wiringPi.h>
#include "oled_common.h"
 #include "oledfont.h" 
#include "oled_interface.h"
#include "bmp.h"
#include "BMM150.h"
 /*
orangepi@orangepizero2:~/wiringOP-master$ gpio readall
 +------+-----+----------+------+---+   H616   +---+------+----------+-----+------+
 | GPIO | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | GPIO |
 +------+-----+----------+------+---+----++----+---+------+----------+-----+------+
 |      |     |     3.3V |      |   |  1 || 2  |   |      | 5V       |     |      |
 |  229 |   0 |    SDA.3 |  OFF | 0 |  3 || 4  |   |      | 5V       |     |      |
 |  228 |   1 |    SCL.3 |  OFF | 0 |  5 || 6  |   |      | GND      |     |      |
 |   73 |   2 |      PC9 |  OFF | 0 |  7 || 8  | 0 | ALT2 | TXD.5    | 3   | 226  |
 |      |     |      GND |      |   |  9 || 10 | 0 | ALT2 | RXD.5    | 4   | 227  |
 |   70 |   5 |      PC6 | ALT5 | 0 | 11 || 12 | 0 | OFF  | PC11     | 6   | 75   |
 |   69 |   7 |      PC5 | ALT5 | 0 | 13 || 14 |   |      | GND      |     |      |
 |   72 |   8 |      PC8 |  OFF | 0 | 15 || 16 | 0 | OFF  | PC15     | 9   | 79   |
 |      |     |     3.3V |      |   | 17 || 18 | 0 | OFF  | PC14     | 10  | 78   |
 |  231 |  11 |   MOSI.1 | ALT4 | 0 | 19 || 20 |   |      | GND      |     |      |
 |  232 |  12 |   MISO.1 | ALT4 | 0 | 21 || 22 | 0 | OFF  | PC7      | 13  | 71   |
 |  230 |  14 |   SCLK.1 | ALT4 | 0 | 23 || 24 | 0 | ALT4 | CE.1     | 15  | 233  |
 |      |     |      GND |      |   | 25 || 26 | 0 | OFF  | PC10     | 16  | 74   |
 |   65 |  17 |      PC1 |  OFF | 0 | 27 || 28 |   |      |          |     |      |
 |  272 |  18 |     PI16 |  OFF | 0 | 29 || 30 |   |      |          |     |      |
 |  262 |  19 |      PI6 |  OFF | 0 | 31 || 32 |   |      |          |     |      |
 |  234 |  20 |     PH10 | ALT3 | 0 | 33 || 34 |   |      |          |     |      |
 +------+-----+----------+------+---+----++----+---+------+----------+-----+------+
 | GPIO | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | GPIO |
 +------+-----+----------+------+---+   H616   +---+------+----------+-----+------+
orangepi@orangepizero2:~/wiringOP-master$ ls

-lwiringPi -lpthread

*/
#define         OLED_SCL_SET_CH1        {digitalWrite(PIN_SCL, HIGH);}
#define         OLED_SCL_CLR_CH1        {digitalWrite(PIN_SCL, LOW);}

#define        OLED_SDA_SET_CH1        {digitalWrite(PIN_SDA, HIGH);}
#define        OLED_SDA_CLR_CH1        {digitalWrite(PIN_SDA, LOW);}
#define        OLED_SDA_IN_CH1         (digitalRead(PIN_SDA))
#define        OLED_SDA_D_OUT_CH1      {pinMode(PIN_SDA, OUTPUT);}
#define        OLED_SDA_D_IN_CH1       {pinMode(PIN_SDA, INPUT);}





static void OLED_I2c_Start(void)
{
    OLED_SDA_D_OUT_CH1;
    OLED_SDA_SET_CH1;
     OLED_SCL_SET_CH1;
	delayMicroseconds(4);	//是否需要延迟
    OLED_SDA_CLR_CH1;
    delayMicroseconds(4);
     OLED_SCL_CLR_CH1;

}
static void OLED_I2c_Stop(void)
{
    OLED_SDA_D_OUT_CH1;   // 设置SDA为输出方向
	 OLED_SCL_CLR_CH1;  
    OLED_SDA_CLR_CH1;
	delayMicroseconds(4);//是否需要延迟
     OLED_SCL_SET_CH1;
    OLED_SDA_SET_CH1;        // 发送结束信号
    delayMicroseconds(4);
    //OLED_SDA_D_IN_CH1;       //设置SDA为输RU方向
}
static u8 VL_IIC_Wait_Ack(void)
{
	int ucErrTime=0;
	OLED_SDA_D_IN_CH1;  //SDA设置为输入  
	//OLED_SDA_SET_CH1;
    delayMicroseconds(1);	   
	 OLED_SCL_SET_CH1;
    delayMicroseconds(1);	 
	while(OLED_SDA_IN_CH1)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			OLED_I2c_Stop();
            
            //printf("time up \n");
			return 1;
		}
	}
	 OLED_SCL_CLR_CH1;//时钟输出0 	   
	return 0;  
}

//产生ACK应答
static void VL_IIC_Ack(void)
{
	 OLED_SCL_CLR_CH1;
	 OLED_SDA_D_OUT_CH1;
	OLED_SDA_CLR_CH1;
	delayMicroseconds(2);
	 OLED_SCL_SET_CH1;
	delayMicroseconds(2);
	 OLED_SCL_CLR_CH1;
}

//不产生ACK应答		    
static void VL_IIC_NAck(void)
{
	 OLED_SCL_CLR_CH1;
	 OLED_SDA_D_OUT_CH1;
	OLED_SDA_SET_CH1;
	delayMicroseconds(2);
	 OLED_SCL_SET_CH1;
	delayMicroseconds(2);
	 OLED_SCL_CLR_CH1;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void VL_IIC_Send_Byte(unsigned char txd)
{                        
    unsigned char t;   
	OLED_SDA_D_OUT_CH1; 	    
     OLED_SCL_CLR_CH1; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7){
            OLED_SDA_SET_CH1;
        }
			
		else{
                OLED_SDA_CLR_CH1;
        }
			
		txd<<=1; 	  
		delayMicroseconds(2);  
		 OLED_SCL_SET_CH1;
		delayMicroseconds(2); 
		 OLED_SCL_CLR_CH1;	
		delayMicroseconds(2);
    }	 
}
static unsigned char VL_IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	OLED_SDA_D_IN_CH1;//
	OLED_SDA_SET_CH1;
	delayMicroseconds(4);;
	for(i=0;i<8;i++ )
	{
		receive<<=1;
		 OLED_SCL_CLR_CH1; 
		delayMicroseconds(4);
		 OLED_SCL_SET_CH1;
		delayMicroseconds(4);
		if(OLED_SDA_IN_CH1)
			receive |= 0x01;   
	  delayMicroseconds(4); //1
	}	
     OLED_SCL_CLR_CH1;	
	return receive;
}


//IIC写一个字节数据
static unsigned char VL_IIC_Write_1Byte(unsigned char SlaveAddress, unsigned char REG_Address,unsigned char REG_data)
{
	OLED_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		OLED_I2c_Stop();//释放总线
		return 1;//没应答则退出

	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();	
	delayMicroseconds(5);
	VL_IIC_Send_Byte(REG_data);
	VL_IIC_Wait_Ack();	
	OLED_I2c_Stop();

	return 0;
}


//IIC读一个字节数据
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	OLED_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack())
	{
		 OLED_I2c_Stop();//释放总线
		 return 1;//没应答则退出
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	 delayMicroseconds(5);
	OLED_I2c_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte();
	OLED_I2c_Stop();

	return 0;
}

//I2C读多个字节数据
static uint8_t VL_I2C_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len)
{
	OLED_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		OLED_I2c_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	 delayMicroseconds(5);
	OLED_I2c_Start(); 
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
	OLED_I2c_Stop();

	return 0;
}

//I2C写多个字节数据
static uint8_t VL_I2C_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len)
{
	OLED_I2c_Start();
	VL_IIC_Send_Byte(SlaveAddress);//发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		OLED_I2c_Stop();//释放总线
		return 1;//没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);
		VL_IIC_Wait_Ack();
	}
	OLED_I2c_Stop();

	return  0;
}


 //发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(u8 dat,u8 mode)
{
	OLED_I2c_Start();
	VL_IIC_Send_Byte(0x78);
	VL_IIC_Wait_Ack();
	if(mode){VL_IIC_Send_Byte(0x40);}
  else{VL_IIC_Send_Byte(0x00);}
	VL_IIC_Wait_Ack();
	VL_IIC_Send_Byte(dat);
	VL_IIC_Wait_Ack();
	OLED_I2c_Stop();
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


//反显函数
void OLED_ColorTurn(u8 i)
{
	if(i==0)
		{
			OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
		}
	if(i==1)
		{
			OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
		}
}

//屏幕旋转180度
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
		{
			OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
			OLED_WR_Byte(0xA1,OLED_CMD);
		}
	if(i==1)
		{
			OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
			OLED_WR_Byte(0xA0,OLED_CMD);
		}
}

//坐标设置

void OLED_Set_Pos(u8 x, u8 y) 
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD);
}   	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<4;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63				 
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 sizey)
{      	
	u8 c=0,sizex=sizey/2;
	u16 i=0,size1;
	if(sizey==8)size1=6;
	else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
	c=chr-' ';//得到偏移后的值
	OLED_Set_Pos(x,y);
	for(i=0;i<size1;i++)
	{
		if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
		if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8字号
		else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16字号
//		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//用户添加字号
		else return;
	}
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示数字
//x,y :起点坐标
//num:要显示的数字
//len :数字的位数
//sizey:字体大小		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 sizey)
{         	
	u8 t,temp,m=0;
	u8 enshow=0;
	if(sizey==8)m=2;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
				continue;
			}else enshow=1;
		}
	 	OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
	}
}
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 sizey)
{
	u8 j=0;
	while (chr[j]!='\0')
	{		
		OLED_ShowChar(x,y,chr[j++],sizey);
		if(sizey==8)x+=6;
		else x+=sizey/2;
	}
}
//显示汉字
void OLED_ShowChinese(u8 x,u8 y,u8 no,u8 sizey)
{
	u16 i,size1=(sizey/8+((sizey%8)?1:0))*sizey;
	for(i=0;i<size1;i++)
	{
		if(i%sizey==0) OLED_Set_Pos(x,y++);
		if(sizey==16) OLED_WR_Byte(Hzk1[no][i],OLED_DATA);//16x16字号
//		else if(sizey==xx) OLED_WR_Byte(xxx[c][i],OLED_DATA);//用户添加字号
		else return;
	}				
}


//显示图片
//x,y显示坐标
//sizex,sizey,图片长宽
//BMP：要显示的图片
void OLED_DrawBMP(u8 x,u8 y,u8 sizex, u8 sizey,u8 BMP[])
{ 	
  u16 j=0;
	u8 i,m;
	sizey=sizey/8+((sizey%8)?1:0);
	for(i=0;i<sizey;i++)
	{
		OLED_Set_Pos(x,i+y);
    for(m=0;m<sizex;m++)
		{      
			OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
		}
	}
} 



//初始化				    
void OLED_Init(void)
{
//初始化wiringPI的库函数
	if(wiringPiSetup()<0) {
		printf("init wiringPi error\n");
	}
	printf("init wiringPi over\n");
	initI2c();          //spi的初始化

	OLED_WR_Byte(0xAE,OLED_CMD); /*display off*/
	OLED_WR_Byte(0x00,OLED_CMD); /*set lower column address*/ 
	OLED_WR_Byte(0x10,OLED_CMD); /*set higher column address*/
	OLED_WR_Byte(0x00,OLED_CMD); /*set display start line*/ 
	OLED_WR_Byte(0xB0,OLED_CMD); /*set page address*/ 
	OLED_WR_Byte(0x81,OLED_CMD); /*contract control*/ 
	OLED_WR_Byte(0xff,OLED_CMD); /*128*/ 
	OLED_WR_Byte(0xA1,OLED_CMD); /*set segment remap*/ 
	OLED_WR_Byte(0xA6,OLED_CMD); /*normal / reverse*/ 
	OLED_WR_Byte(0xA8,OLED_CMD); /*multiplex ratio*/ 
	OLED_WR_Byte(0x1F,OLED_CMD); /*duty = 1/32*/ 
	OLED_WR_Byte(0xC8,OLED_CMD); /*Com scan direction*/ 
	OLED_WR_Byte(0xD3,OLED_CMD); /*set display offset*/ 
	OLED_WR_Byte(0x00,OLED_CMD); 
	OLED_WR_Byte(0xD5,OLED_CMD); /*set osc division*/ 
	OLED_WR_Byte(0x80,OLED_CMD); 
	OLED_WR_Byte(0xD9,OLED_CMD); /*set pre-charge period*/ 
	OLED_WR_Byte(0x1f,OLED_CMD); 
	OLED_WR_Byte(0xDA,OLED_CMD); /*set COM pins*/ 
	OLED_WR_Byte(0x00,OLED_CMD); 
	OLED_WR_Byte(0xdb,OLED_CMD); /*set vcomh*/ 
	OLED_WR_Byte(0x40,OLED_CMD); 
	OLED_WR_Byte(0x8d,OLED_CMD); /*set charge pump enable*/ 
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_Clear();
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
}


  int main_1()
{
    
}

void set_callback_function(char* _msg ,void(*pfunc)(char*))
{
   (*pfunc)(_msg);
   return ;
}
void oled_task(void (*func)(char*))
{
	unsigned char buffer[6];
	OLED_Init();//初始化OLED 
	OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(1);//0正常显示 1 屏幕翻转显示	

	char res =I2cinit(I2C_ADDRESS_4);
	if(res)
	printf("open i2c ok \n");
	setOperationMode(BMM150_POWERMODE_NORMAL);
	setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
	setRate(BMM150_DATA_RATE_10HZ);
	setMeasurementXYZ(MEASUREMENT_X_ENABLE,MEASUREMENT_Y_ENABLE,MEASUREMENT_Z_ENABLE);
	while(1){
	//sBmm150MagData_t magData = getGeomagneticData();
	OLED_ShowString(8,0,"Robot compass",16);
	OLED_ShowString(0,2,"Degree:",16);  
	float compassDegree = getCompassDegree();
	OLED_ShowNum(80,2,(unsigned int)compassDegree,3,16);
	usleep(5000);
	}
}
