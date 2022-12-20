#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>   //low_level i/o
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <sys/select.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>

#include "hardwareserial.h"
HardwareSerial::HardwareSerial() {

   _uart_fd =0;
}
speed_t HardwareSerial:: getBaudrate(int baudrate)
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

void HardwareSerial::begin(char *dev ,unsigned long baud)
{
 
	struct termios oldtio,newtio;   
	speed = getBaudrate(baud);    
	_uart_fd=open(dev,O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY);    
	if(_uart_fd<0)    
	{        
		perror(dev);  
		
		//exit(1);    
	}    //save to oldtio    
	tcgetattr(_uart_fd,&oldtio);    //clear newtio   
	bzero(&newtio,sizeof(newtio));    
	//newtio.c_cflag = speed|CS8|CLOCAL|CREAD|CRTSCTS;   
	newtio.c_cflag = speed|CS8|CLOCAL|CREAD;   //PARENB
	newtio.c_iflag = IGNPAR;      
	newtio.c_oflag = 0;    
  
	tcflush(_uart_fd,TCIFLUSH);      
	tcsetattr(_uart_fd,TCSANOW,&newtio);      
	tcgetattr(_uart_fd,&oldtio); 
}

void HardwareSerial::setDTR()
{
     

    uint32_t dtr_bit = TIOCM_DTR;
    ioctl(_uart_fd, TIOCMBIS, &dtr_bit);
}

void HardwareSerial::clearDTR()
{
     

    uint32_t dtr_bit = TIOCM_DTR;
    ioctl(_uart_fd, TIOCMBIC, &dtr_bit);
}

void HardwareSerial::end(bool turnOffDebug)
{
    if(_uart_fd >0 ) {
        close(_uart_fd);
    }
   
}

int HardwareSerial::peek(void)
{

}

int HardwareSerial::read_one_byte()
{
    unsigned char  oneByte;
    if(_uart_fd > 0) {
         if(read(_uart_fd,&oneByte,1)> 0) 
			 return oneByte;
    }
    
}

// read characters into buffer
// terminates if size characters have been read, or no further are pending
// returns the number of characters placed in the buffer
// the buffer is NOT null terminated.
size_t HardwareSerial::read_buffer(uint8_t *buffer, size_t size)
{

    size_t count = 0;
    count = read(_uart_fd,buffer,size);
    return count;
}

void HardwareSerial::flush(void)
{
 
}

void HardwareSerial::flush(bool txOnly)
{
 
}

size_t HardwareSerial::write_one_byte(uint8_t c)
{
    write(_uart_fd,(uint8_t *)&c,1);  
    return 1;
}

size_t HardwareSerial::write_buffer(const uint8_t *buffer, size_t size)
{
    write(_uart_fd, buffer, size);
    return size;
}
uint32_t  HardwareSerial::baudRate()

{
	return this->speed;
}



