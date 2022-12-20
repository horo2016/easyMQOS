#include"Uart_comm.h"


 speed_t getBaudrate(int baudrate)
{	
switch(baudrate) 
	{	
	case 0: return B0;	
	case 50: return B50;	
	case 75: return B75;	
	case 110: return B110;	
	case 134: return B134;	
	case 150: return B150;	
	case 200: return B200;	
	case 300: return B300;	
	case 600: return B600;	
	case 1200: return B1200;	
	case 1800: return B1800;	
	case 2400: return B2400;	
	case 4800: return B4800;	
	case 9600: return B9600;	
	case 19200: return B19200;	
	case 38400: return B38400;	
	case 57600: return B57600;	
	case 115200: return B115200;	
	case 230400: return B230400;	
	case 460800: return B460800;	
	case 500000: return B500000;	
	case 576000: return B576000;	
	case 921600: return B921600;	
	case 1000000: return B1000000;	
	case 1152000: return B1152000;	
	case 1500000: return B1500000;	
	case 2000000: return B2000000;	
	case 2500000: return B2500000;	
	case 3000000: return B3000000;	
	case 3500000: return B3500000;	
	case 4000000: return B4000000;	
	default: return -1;	
	}
}

 int OpenDev(char *Dev,int baudrate)
{	
	speed_t speed;    
	  
	int fdt;    
	struct termios oldtio,newtio;   
	speed = getBaudrate(baudrate);    
	fdt=open(Dev,O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY);    
	if(fdt<0)    
	{        
		perror(Dev);  
		return -1;
		//exit(1);    
	}    //save to oldtio    
	tcgetattr(fdt,&oldtio);    //clear newtio   
	bzero(&newtio,sizeof(newtio));    
	//newtio.c_cflag = speed|CS8|CLOCAL|CREAD|CRTSCTS;   
	newtio.c_cflag = speed|CS8|CLOCAL|CREAD;   //PARENB
	newtio.c_iflag = IGNPAR;      
	newtio.c_oflag = 0;    
	//printf("newtio.c_cflag=%x\n",newtio.c_cflag);   
	tcflush(fdt,TCIFLUSH);      
	tcsetattr(fdt,TCSANOW,&newtio);      
	tcgetattr(fdt,&oldtio);    //printf("oldtio.c_cflag=%x\n",oldtio.c_cflag);   
	return fdt;
}
 int OpenDev_insulation(char *Dev,int baudrate)
{	
	speed_t speed;    
	    
	int fdt;    
	struct termios oldtio,newtio;   
	speed = getBaudrate(baudrate);    
	fdt=open(Dev,O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY);    
	if(fdt<0)    
	{        
		perror(Dev);        
		exit(1);    
	}    //save to oldtio    
	tcgetattr(fdt,&oldtio);    //clear newtio   
	bzero(&newtio,sizeof(newtio));    
	//newtio.c_cflag = speed|CS8|CLOCAL|CREAD|CRTSCTS;   
	newtio.c_cflag = speed|CS8|CLOCAL|CREAD|PARENB;   //
	newtio.c_iflag = IGNPAR;      
	newtio.c_oflag = 0;    
	//printf("newtio.c_cflag=%x\n",newtio.c_cflag);   
	tcflush(fdt,TCIFLUSH);      
	tcsetattr(fdt,TCSANOW,&newtio);      
	tcgetattr(fdt,&oldtio);    //printf("oldtio.c_cflag=%x\n",oldtio.c_cflag);   
	return fdt;
}
 void GetCrC(unsigned char *CmmBuf, unsigned char Len)
{
    unsigned short crc = 0xFFFF;
    unsigned char i;
    unsigned char j;
    if(Len > 78) return;
    j = 0;
    while(j < Len)
    {
        crc ^= CmmBuf[j];
        for(i=0; i < 8; i++)
        {
            if(crc & 0x01)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
        j++;
    }
    CmmBuf[j] = crc % 0x100; //校验和从长度算起，长度为长度减3
    CmmBuf[j + 1] = crc / 0x100;


}

