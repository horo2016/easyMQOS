#ifndef UART_COMM_H
#define UART_COMM_H

#include "sys/socket.h"
#include "netinet/in.h"

#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <sys/stat.h>
#include <termios.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>



extern  speed_t getBaudrate(int baudrate);
extern int OpenDev(char *Dev,int baudrate);
extern  int OpenDev_insulation(char *Dev,int baudrate);
extern  void GetCrC(unsigned char *CmmBuf, unsigned char Len);
#endif

