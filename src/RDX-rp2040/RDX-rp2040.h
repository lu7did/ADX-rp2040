//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              RDX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 2.0
//
// This is a direct port into the rp2040 architecture of the ADX_UNO firmware code (baseline version 1.1).
// Version 2.0 is an enhanced version which extends the capabilities of the ADX-rp2040 firmware
//*********************************************************************************************************
/*---------------------------------------------------------------------------------------------------*
 * Includes and macro definitions for the ADX-rp2040 transceiver firmware                            *
 *---------------------------------------------------------------------------------------------------*/
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                       External libraries used                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
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

/*------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "RDX"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION "2.0"
#define BUILD   "41"
/*-------------------------------------------------
 * Macro expansions
 */
#define digitalWrite(x,y) gpio_put(x,y)
#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0
typedef void (*CALLBACK)();
typedef void (*CALLQSO)(int i);
/*--------------------------------------------------
 * Program configuration parameters
 */
#define DEBUG                1  //Uncomment to activate debugging traces (_INFOLIST(...) statements thru _SERIAL

#undef  UART
#define BAUD            115200
#define FSK_IDLE          1000    //Standard wait without signal
#define FSK_ERROR            4
#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            3000    //Maximum FSK frequency computed
#define FSK_USEC       1000000    //Constant to convert T to f
#define VOX_MAXTRY          15    //VOX control cycles


/*----
   Output control lines
*/
#define RX              2  //RX Switch

/*---
   LED
*/
#define WSPR            7  //WSPR LED
#define JS8             6  //JS8 LED
#define FT4             5  //FT4 LED
#define FT8             4  //FT8 LED

#define TX              3  //TX LED

/*---
   Switches
*/
#define UP             10  //UP Switch
#define DOWN           11  //DOWN Switch
#define TXSW            8  //RX-TX Switch


/*---
 * 
 */
  
#define BUTTON_TX       0
#define BUTTON_CQ       1
#define BUTTON_AUTO     2
#define BUTTON_BAND     3
  
#define BUTTON_END      4


/*---
   Signal input pin
*/

#define FSKpin         27  //Frequency counter algorithm, signal input PIN (warning changes must also be done in freqPIO)
#define ADC0           26  //Audio received (centered at Vcc/2)
#define ADC_CHANNEL     0
/*---
    I2C
*/
#define I2C_SDA        16  //I2C SDA
#define I2C_SCL        17  //I2C SCL

#define SI5351_REF     25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz

/*----
   Autocalibration pin
*/
#define CAL             9      //Automatic calibration entry

/*----------------------------------------------------
 * ft8 definitions
 */
//Created by AA1GD Aug. 25, 2021
//OCTOBER 18, 2021 AT 5:14 PM CDT FIRST ON AIR DECODES WITH THIS

#define MY_CALLSIGN "LU2EIC"
#define MY_GRID "GF05"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GENERAL PURPOSE GLOBAL DEFINITIONS                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BANDS          4            //Max number of bands allowed
#define MAXBAND        9

/*-----------------------------------------------------
   Definitions for autocalibration

*/
#define AUTOCAL             1
#define CAL_STEP          500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define CAL_COMMIT         12
#define CAL_ERROR           1

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         External References                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
extern char hi[128];
extern unsigned long freq;
extern bool stopCore1;
extern char my_callsign[16];
extern char my_grid[8];
extern Si5351 si5351;
extern char timestr[12];
extern bool qwait;
extern int magint[960];
extern int TX_State;
extern char programname[12];
extern char version[6];
extern char build[6];
extern char ip[16];
extern uint8_t state;
extern bool autosend;
extern uint8_t nTry;
extern bool triggerCQ;
extern bool triggerCALL;
extern uint16_t call_af_frequency;
extern int8_t call_self_rx_snr;
extern char call_station_callsign[8];
extern char call_grid_square[4];
extern uint16_t call_qsowindow;
extern int Band_slot;
extern const uint16_t Bands[BANDS];

extern CALLBACK fftReady;
extern CALLQSO  qsoReady;
extern CALLBACK fftEnd;
extern CALLBACK  txIdle;

extern void initSi5351();
extern void AutoCalibration();
extern void timeSync();
extern int getQSOwindow();
extern void updateEEPROM();
extern void tft_updatewaterfall(int m[]);
extern void setCALL();
extern void INIT();
extern void Calibration();
extern void Band_Select();
extern void ManualTX();
extern void Band_assign();
extern void Freq_assign();
extern void Mode_assign();
extern void checkButton();
extern void tft_setup();
extern void tft_run();
extern void tft_endoftime();
extern void tft_checktouch();
extern void tft_resetBar();
extern void tft_endQSO();
extern void tft_setBar(int colour);
extern void tft_storeQSO(uint16_t qsowindow, uint16_t _qso,char *s,uint16_t af_frequency,int8_t self_rx_snr,char *station_callsign,char *grid_square);
extern unsigned long Slot2Freq(int s);
extern void tft_updateBand();



extern void tft_setBarTx();

extern void tft_set(int btnIndex,int v);

extern void startTX();
extern void stopTX();

/*
extern void tft_setTX(bool t);
*/

extern time_t t_ofs;
extern time_t now;


extern struct tm timeinfo;        //current time
extern struct tm timeprev;        //epoch time

/*-------------------------------------------------------
 * Debug and development aid tracing
 * only enabled if DEBUG is defined previously
 */
#ifdef DEBUG

#ifdef  UART    //Can test with the IDE, USB based, serial port or the UART based external serial port
#define _SERIAL Serial1
#else
#define _SERIAL Serial
#endif //UART

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
#else //!DEBUG
#define _INFOLIST(...)
#endif


/*----------------------------------------------------------
 * Extern functions across the different sub-systems
 */

extern uint16_t ft8_crc(const uint8_t message[], int num_bits);
