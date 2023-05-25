#include "sonars.h"


#include <stdio.h>
#include <unistd.h>
 
#include <pthread.h>

#include <sched.h>
 
#include <string.h>
#include <stdint.h>
#include <iostream> 
#include <string>
#include <vector>
#include "osp_syslog.h"
#include"wiringPi.h"
 
#include <stdlib.h>

#include <assert.h>
#include <sys/select.h>
 
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>





#define  TRIG_C7 13 //PC7
#define  ECHO_C8 8  //PC8

#define  TRIG_C9 2 //PC9
#define  ECHO_C10 16  //PC10

#define TRIG_C5  7   //PC5
#define ECHO_C6  5   //PC6

 

class Sonar {
public:
    // wiringPi GPIO pins
    int trigger_pin;
    int echo_pin;
    int id;

    uint32_t start_tick;
    uint32_t elapsed_ticks;

    std::string frame;
     

    bool range_error = false;

     
    Sonar(int trigger_pin, int echo_pin, int id)
    {
        this->trigger_pin = trigger_pin;
        this->echo_pin = echo_pin;
        this->id = id;

        start_tick = 0;
        elapsed_ticks = 0;

       
    }

  /*  void range_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (range_error) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Range out of bounds!");
        }
        else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Range within bounds!");
        }
    }*/
};


static std::vector<Sonar> sonars;
#define HC_SR04_READ_TIMEOUT 100000 //us -> 100ms

static unsigned int  time_spent = 0;
 
static void hc_sr04_clear_timeout(void)
{
    time_spent = 0;
}

static unsigned char  hc_sr04_is_timeout(unsigned short sleep_us)
{
    time_spent += sleep_us;

    unsigned char is_timeout = 1;
    if (time_spent >= HC_SR04_READ_TIMEOUT) {
        is_timeout = 0;
    }

    return is_timeout;
}
/* Trigger the next sonar */
void sonar_trigger(int sonar)
{
    int pin = sonars[sonar].trigger_pin;

	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);//trig
	delayMicroseconds(10);
	//发出超声波脉冲
	digitalWrite(pin, LOW);

}




/* Handle pin change */
float echo_callback( uint32_t id)
{
	struct timeval tv1;
	struct timeval tv2;
	long start, stop;
	int dis;
	int pin = sonars[id].echo_pin;
    hc_sr04_clear_timeout();
	while((digitalRead(pin) != 1) && (hc_sr04_is_timeout(5))){
	      usleep(5);
	}
	gettimeofday(&tv1, NULL);
	hc_sr04_clear_timeout();
	while((digitalRead(pin) != 0) && (hc_sr04_is_timeout(100))){
	  usleep(100);
	}
	gettimeofday(&tv2, NULL);
	 
	start = tv1.tv_sec * 1000000 + tv1.tv_usec;
//	DEBUG(LOG_DEBUG, "start:%d \n",start);
	//微秒级的时间
	stop = tv2.tv_sec * 1000000 + tv2.tv_usec;
//	DEBUG(LOG_DEBUG, "stop:%d \n",stop);
//	DEBUG(LOG_DEBUG, "stop-start:%d \n",(stop-start));//34cm/ms
	if((stop - start) >= 38000)
	      return 5.0;
//	DEBUG(LOG_DEBUG, "distance:%d cm\n",(stop - start)  * 34 / 2000);
	dis = (stop - start)  * 34 / 2000;// 
	 
	return  (float)dis;
     
     
     
}

int setup_gpio()
{
    wiringPiSetup();
	
    for (const auto& sonar : sonars) {
	
        
		pinMode (sonar.echo_pin,INPUT ) ;//echo
		pinMode (sonar.trigger_pin,OUTPUT ) ; 
		digitalWrite(sonar.trigger_pin, LOW);
	    
      
    }

    return true;
}

 

 
 void set_callback_function(char* _msg ,void(*pfunc)(char*))
{
   (*pfunc)(_msg);
   return ;
}
void sonars_task(void (*func)(char*))
 {
	 
 	 // pin numbers are specific to the hardware
	 sonars.push_back(Sonar(TRIG_C5, ECHO_C6, 0));
	 sonars.push_back(Sonar(TRIG_C7, ECHO_C8, 1));
	 sonars.push_back(Sonar(TRIG_C9, ECHO_C10, 2));
	// sonars.push_back(Sonar(27, 22, 3));
	// sonars.push_back(Sonar(19, 26, 4));
 
	 if (!setup_gpio()) {
		 printf("Cannot initalize gpio");
		 return  ;
	 }
 

    unsigned short buffer[3];
	 while (1) {
		 for (auto& sonar: sonars) {
            sonar_trigger(sonar.id);
            if(sonar.id == 0)
               buffer[sonar.id] =  echo_callback(sonar.id)  ;
            else if(sonar.id == 1)
             buffer[sonar.id] =    echo_callback(sonar.id)  ;
            else if(sonar.id == 2)
             buffer[sonar.id] =	 echo_callback(sonar.id)  ;

            printf("%d dis:%d \n",sonar.id,buffer[sonar.id] );
             
            usleep(1000);
		 }
		 set_callback_function((char *) buffer,func);
	 }
  
 
	 return ;
 }

 