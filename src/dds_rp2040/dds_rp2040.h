//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              dds_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <si5351.h>
#include "hardware/watchdog.h"
#include "Wire.h"
#include <EEPROM.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include <WiFi.h>
#include <Time.h>
#include <stdbool.h>
*/

//#include "piodco.h"
/*-------------------------------------------------
 * Macro expansions
 */
#define digitalWrite(x,y) gpio_put(x,y)
#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0
/*--------------------------------------------------
 * Program configuration parameters
 */
#undef  UART

#define DEBUG                1  //Uncomment to activate debugging traces (_INFOLIST(...) statements thru _SERIAL
#define BAUD            115200
#define CAT                  1  //Uncomment to activate CAT
#define STACK_SIZE      11000   //Bytes

/****************************************************************************  
* RPi pico specific defs.
****************************************************************************/  
#define CLK_ADC_FREQ 48000000L                 /* pico's ADC clock is 48M. */
#define GPIO(x) (x)                        /* pico GPIO number to address. */
#define PLL_SYS_MHZ 270                                   /* system clock. */


/*-------------------------------------------------------------*
   CPU Configuration parameters
  -------------------------------------------------------------*/
#define MULTICORE       1       //Processing is split between core0 and core1
extern struct semaphore spc;      //Semaphore to protect multithread access to the signal queue
extern char timestr[12];
#define TIMEZONE 0.00
extern char hi[80];


//extern int PioDCOInit(PioDco *pdco, int gpio, int cpuclkhz);
/*
extern int PioDCOSetFreq(PioDco *pdco, uint32_t ui32_frq_hz, int32_t ui32_frq_millihz);
extern int32_t PioDCOGetFreqShiftMilliHertz(const PioDco *pdco, uint64_t u64_desired_frq_millihz);
extern void PioDCOStart(PioDco *pdco);
extern void PioDCOStop(PioDco *pdco);
extern void RAM (PioDCOWorker2)(PioDco *pDCO);
extern void RAM (PioDCOWorker)(PioDco *pDCO);
extern void PioDCOSetMode(PioDco *pdco, enum PioDcoMode emode);
*/










#ifdef DEBUG
#define _SERIAL Serial

#define _INFOLIST(...) \
  do { \
    now = time(0) - t_ofs;  \
    gmtime_r(&now, &timeinfo);  \
    sprintf(timestr,"[%02d:%02d:%02d] ",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);   \
    strcpy(hi,timestr); \
    strcat(hi,"@");  \
    sprintf(hi+strlen(hi),__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)

#define _INFO(...) \
  do { \
    while(!sem_try_acquire(&spc)); \
    now = time(0) - t_ofs;  \
    gmtime_r(&now, &timeinfo);  \
    sprintf(timestr,"[%02d:%02d:%02d] ",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);   \
    strcpy(hi,timestr); \
    strcat(hi,"@");  \
    sprintf(hi+strlen(hi),"%s ",__func__); \
    sprintf(hi+strlen(hi),__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
    sem_release(&spc); \
  } while (false)

/*-----------------------------------------------
   IPC protected serial write to be used when
   debugging MULTICORE
*/

#ifdef MULTICORE
#define _print(...) \
  do { \
    while(!sem_try_acquire(&spc)); \
    rp2040.idleOtherCore(); \
    _SERIAL.print(__VA_ARGS__); \
    _SERIAL.flush(); \
    sem_release(&spc); \
    rp2040.resumeOtherCore(); \
  } while (false)
#define _println(...) \
  do { \
    while(!sem_try_acquire(&spc)); \
    rp2040.idleOtherCore(); \
    _SERIAL.println(__VA_ARGS__); \
    _SERIAL.flush(); \
    sem_release(&spc); \
    rp2040.resumeOtherCore(); \
  } while (false)
#endif //MULTICORE

#else //!DEBUG
#define _INFOLIST(...)
#define _INFO(...)
#define _SERIAL Serial
#define _println(...)
#define _print(...)



#endif



#define GEN_FRQ_HZ 29977777L
 

/*-----------------------------------------------------
   Definition of CAT (comment out if not desired)
*/

/*--------------------------------------------------
   Program configuration parameters
*/

