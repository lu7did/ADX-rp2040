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
   Signal input pin
*/

#define FSKpin         27  //Frequency counter algorithm, signal input PIN (warning changes must also be done in freqPIO)
#define AFpin          26  //Audio received (centered at Vcc/2)
/*---
    I2C
*/
#define I2C_SDA        16  //I2C SDA
#define I2C_SCL        17  //I2C SCL

#define SI5351_REF     25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz

/*----------------------------------------------------
 * ft8 definitions
 */
//Created by AA1GD Aug. 25, 2021
//OCTOBER 18, 2021 AT 5:14 PM CDT FIRST ON AIR DECODES WITH THIS

#define MY_CALLSIGN "LU7DZ"
#define MY_GRID "GF05"
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         External References                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
extern char hi[128];
extern unsigned long freq;
extern bool stopCore1;

extern void INIT();
extern void Calibration();
extern void Band_Select();
extern void ManualTX();
extern void Band_assign();
extern void Freq_assign();
extern void Mode_assign();

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
    strcpy(hi,"@"); \
    sprintf(hi+1,__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)
//#define _INFOLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.write(hi);_SERIAL.flush();
//#define _INFOLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.write(hi);_SERIAL.flush();
#else //!DEBUG
#define _INFOLIST(...) (void)0
#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness

/*----------------------------------------------------------
 * Extern functions across the different sub-systems
 */

extern uint16_t ft8_crc(const uint8_t message[], int num_bits);
