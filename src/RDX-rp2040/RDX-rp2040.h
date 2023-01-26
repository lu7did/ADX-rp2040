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
#define BUILD   "53"
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
#define RP2040_W             1  //Comment if running on a standard Raspberry Pico (non Wireless)

#if defined(RP2040_W)
#define FSBROWSER            1  //Comment out if a File System browser is not needed
#endif //RP2040_W

#if defined(FSBROWSER)
#define ADIF                 1  //Comment out if an ADIF logging is not needed
#endif //FSBROWSER

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

#define MY_CALLSIGN "LU2EIC"
#define MY_GRID "GF05"

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                               WiFi Access Point credentials                                           *
//* Replace the AP SSID and password of your choice, if not modified the firmware won't be able to sync   *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#ifndef WIFI_SSID
//The ap.h file must contain valid Wifi credentials following the format
//#define WIFI_SSID                  "Your WiFi SSID"
//#define WIFI_PSK                   "0123456789"
//You might replace that include with a couple of #define for WIFI_SSID and WIFI_PSK for your Wifi AP
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#if __has_include("ap.h")
#include "ap.h"
#else
#include "Y:\Documents\GitHub\ap.h"        //This is a trick to provide credentials on a file outside of the GitHub package
                                           //This is an horrendous practice that should be avoided, the only reason is
                                           //not to forget and publish my WiFi credentials (I test with)
                                           //Once you create your own ap.h file just place it at the folder where the rest
                                           //of the firmware is and replace the include with
                                           //     #include "ap.h"
#endif                                           
#endif //WIFI_SSID

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GENERAL PURPOSE GLOBAL DEFINITIONS                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BANDS          4            //Max number of bands allowed
#define MAXBAND        9
#define AF_FREQ     1500

/*-----------------------------------------------------
   Definitions for autocalibration

*/
#define AUTOCAL             1
#define CAL_STEP          500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define CAL_COMMIT         12
#define CAL_ERROR           1

/*------------------------------------------------------
 * EEPROM Address Map
 */
#define EEPROM_ADDR_CAL     10       //CALB  int32_t
#define EEPROM_ADDR_TEMP    30
#define EEPROM_ADDR_MODE    40       //MODE  int
#define EEPROM_ADDR_SLOT    50       //SLOT  int

#define EEPROM_ADDR_BUILD   60       //      int
#define EEPROM_ADDR_MYCALL  70       //MYCALL  char[16]
#define EEPROM_ADDR_MYGRID  80       //GRID  char[8]
#define EEPROM_ADDR_SSID    90       //SSID  char[40]
#define EEPROM_ADDR_PSK    130       //PSK   char[16]
#define EEPROM_ADDR_MAXTX  140       //TX    
#define EEPROM_ADDR_MAXTRY 150       //TRY
#define EEPROM_ADDR_AUTO   160       //AUTO
#define EEPROM_ADDR_HOST   170       //HOST
#define EEPROM_ADDR_PORT   180       //PORT

#define EEPROM_ADDR_END    250

/*------------------------------------------------------------
 * GUI Icon enumeration
 */
#define MAXICON  9

#define WIFIICON 0
#define TERMICON 1
#define CALICON  2
#define CNTICON  3
#define CATICON  4
#define WSJTICON 5
#define QUADICON 6
#define MUTEICON 7
#define SPKRICON 8


#define SLOT_1               1
#define SLOT_2               2
#define SLOT_3               3
#define SLOT_4               4

//#include "ap.h"        //This is a trick to provide credentials on a file outside of the GitHub package

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                               TCP/IP related areas                                                    *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#define HOSTNAME              PROGNAME
#define NTP_SERVER1           "132.163.97.1"    //time.nist.gov in case of a faulty DNS server
#define NTP_SERVER2           "pool.ntp.org"     //NTP server secondary      
#define INET_SERVER           "www.google.com"     //Check some relevant host or IP address of interest to check connectivity
#define TIMEZONE              -3                 //Buenos Aires, Argentina
#define ADIFFILE              "/rdx.txt"
#define QSO_MESSAGE           "73 and GL"


#define SYNC_OK               0
#define SYNC_NO_INET          1
#define SYNC_NO_AP            2
#define SYNC_NO_NTP           3


#define WIFI_TOUT            20        //WiFi connection timeout (in Secs)
#define UDP_PORT           2237        //UDP Port to listen
#define TCP_PORT           9000        //TCP Port to listen for connections
#define UDP_BUFFER          256


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         External References                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
extern Si5351 si5351;

extern char my_callsign[16];
extern char my_grid[8];
extern uint8_t nTry;
extern int Band_slot;
extern const uint16_t Bands[BANDS];

#ifdef RP2040_W
extern char wifi_ssid[40];
extern char wifi_psk[16];
extern char hostname[32];
extern int  tcp_port;
#endif //RP2040_W


/*------------------------------------------------------------------
 * System global state variables
 */
extern bool autosend;
extern bool stopCore1;
extern bool qwait;
extern bool triggerCQ;
extern bool triggerCALL;
extern bool enable_adc;
extern bool DSP_flag;
extern bool logADIF;
extern bool endQSO;


extern char hi[128];
extern unsigned long freq;
extern char timestr[12];
extern int magint[960];
extern int TX_State;
extern char programname[12];
extern char version[6];
extern char build[6];
extern char ip[16];
extern uint8_t ft8_state;
extern uint16_t call_af_frequency;
extern int8_t call_self_rx_snr;
extern char call_station_callsign[8];
extern char call_grid_square[4];
extern uint16_t call_qsowindow;

/*---------------------------------------------------
 * Time related variables
 */
extern time_t t_ofs;
extern time_t now;
extern struct tm timeinfo;        //current time
extern struct tm timeprev;        //epoch time


extern char ntp_server1[32];    //Server defined to sync time (primary)
extern char ntp_server2[32];    //Server defined to sync time (secondary)
extern char inet_server[32];    //Server defined to validate Inet connectivity

extern CALLBACK fftReady;
extern CALLQSO  qsoReady;
extern CALLBACK fftEnd;
extern CALLBACK  txIdle;

extern void setup_FSBrowser();
extern void loop_FSBrowser();

extern void tft_FSBrowser();
extern void tft_autocal();
extern void tft_error(uint16_t e);
extern void tft_print(char *t);
extern void tft_process();
extern void tft_updatewaterfall(int m[]);
extern void tft_setup();
extern void tft_run();
extern void tft_endoftime();
extern void tft_checktouch();
extern void tft_resetBar();
extern void tft_endQSO();
extern void tft_init();
extern void tft_setBar(int colour);
extern void tft_setIP(char *ip_address);
extern void tft_storeQSO(uint16_t qsowindow, uint16_t _qso,char *s,uint16_t af_frequency,int8_t self_rx_snr,char *station_callsign,char *grid_square);
extern void tft_updateBand();
extern void tft_setBarTx();
extern void tft_set(int btnIndex,int v);
extern void tft_ADIF();
extern void tft_syncNTP();
extern void tft_quad();
extern void tft_ADIF();

extern void tft_iconState(int _icon,bool _state);
extern void tft_iconSet(int _icon,bool _enabled);
extern void tft_iconActive(int _icon,bool _active);


extern int checkAP(char* s, char* p);
extern void resetAP();
extern int setup_wifi();
extern bool getClock(char* n1, char* n2);

extern int writeQSO(char *adifFile,char *call,char *grid, char *mode, char *rst_sent,char *rst_rcvd,char *qso_date,char *time_on, char *band, char *freq, char *mycall, char *mygrid, char *message);



extern void initSi5351();

extern void AutoCalibration();
extern void timeSync();
extern int getQSOwindow();

extern void updateEEPROM();
extern void setCALL();
extern void INIT();
extern void Calibration();
extern void Band_Select();
extern void Band2Str(char *str);
extern void ManualTX();
extern void Band_assign();
extern void Freq_assign();
extern void Mode_assign();
extern void checkButton();
extern unsigned long Slot2Freq(int s);
extern void setup_adif();

extern void startTX();
extern void stopTX();


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
#define _SERIAL Serial
#endif


/*----------------------------------------------------------
 * Extern functions across the different sub-systems
 */

extern uint16_t ft8_crc(const uint8_t message[], int num_bits);
