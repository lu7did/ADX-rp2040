//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 2.1
//
// This is a direct port into the rp2040 architecture of the ADX_UNO firmware code (baseline version 1.1).
// Version 2.0 is an enhanced version which extends the capabilities of the ADX-rp2040 firmware
// Version 2.1 add CAT support (matching ADX_CAT_V1.4)
//*********************************************************************************************************
//
//********************************[ CAT CONTROL SETTINGS and CAT Functionality ]********************
// CAT CONTROL RIG EMULATION: KENWOOD TS2000
// SERIAL PORT SETTINGS: 115200 baud,8 bit,1 stop bit
// When CAT is active FT4 and JS8 leds will be solid lit.
// In CAT mode none of the Switches and leds are active including TX SWITCH in order to avoid different setting clashes except TX LED. 
// TX LED WILL BE LIT briefly on/off and then solid during TX WHEN TRANSMITTING IN CAT Mode.
// In CAT mode ADX can be controlled ONLY by CAT interface. Frequency and TX can be controlled via CAT.
// To get out of CAT mode and to use ADX with Switch and led interface just recycle power. Once activated CAT mode stays active as rig control Until power recycle. 
// In CAT mode manual Band setup is deactivated and ADX can be operated on any band as long as the right lpf filter module is plugged in. 
// IN CAT MODE MAKE SURE THE CORRECT LPF MODULE IS PLUGGED IN WHEN OPERATING BAND IS CHANGED!!! IF WRONG LPF FILTER MODULE IS PLUGGED IN then PA POWER MOSFETS CAN BE DAMAGED

/*---------------------------------------------------------------------------------------------------*
 * Includes and macro definitions for the ADX-rp2040 transceiver firmware                            *
 *---------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "ADX_rp2040"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  "2.0"
#define BUILD     "00"

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

#define BAUD            115200
#define CAT                  1  //Uncomment to activate CAT
//#define DDSPIO               1  //ddsPIO activated (comment to activate Si5351 clock) 
//#define PICOW                1  //Define where the runtime is using a Pico-W board 
//#define UART                 1  //Uncomment to use UART0 as the main output
//#define DEBUG                1  //Uncomment to activate debugging traces (_INFOLIST(...) statements thru _SERIAL

/*---------------------------------------------------
 * Define band support
*/
#define NBANDS        7
#define NMODES        4
#define SLOT          4
/*-----------------------------------------------------
   Definition of CAT (comment out if not desired)
*/

/*--------------------------------------------------
   Audio FSK processing parameters
*/

#define FSK_IDLE          1000    //Standard wait without signal
#define FSK_ERROR            4
#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            3000    //Maximum FSK frequency computed
#define FSK_USEC       1000000    //Constant to convert T to f
#define VOX_MAXTRY          15    //VOX control cycles

/*-----------------------------------------------------
   Definitions for autocalibration

*/
#define AUTOCAL             1
#define CAL_STEP          500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define CAL_COMMIT         12
#define CAL_ERROR           1

/*-----------------------------------------------------
 * ddsPIO definitions
 */

#define DDSPIO               1                  //Use soft rp2040-based DDS
#define OVERCLOCK            1                  //Users extended clock range to improve PLL error at high frequencies
#define PLL_BASE      12000000                  //PLL
#define DDSFREQ       14000000                  //Initial DDS frequency

//***********************************************************************************************
//* The following defines the ports used to connect the hard switches (UP/DOWN/TX) and the LED
//* to the rp2040 processor in replacement of the originally used for the ADX Arduino Nano or UNO
//* (see documentation for the hardware schematic and pinout
//***********************************************************************************************
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

/*-------
   Define pinout for CLK0 and CLK1 when using the rp2040 as a clock generator (DDS)
*/   

#ifdef DDSPIO
#define DDS_CLK0        0
#define DDS_CLK1        1
#endif //DDSPIO

#ifndef DDSPIO
/*---
    I2C
*/
#define I2C_SDA        16  //I2C SDA
#define I2C_SCL        17  //I2C SCL
#endif //!DDSPIO

/*---
    CAT
*/
#define UART_TX        12
#define UART_RX        13

/*----
   Autocalibration pin
*/
#ifdef AUTOCAL
#define CAL             9      //Automatic calibration entry
#endif //AUTOCAL
//**********************************************************************************************

#ifndef DDSPIO
#define SI5351_REF 		25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#endif //!DDSPIO

/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
extern volatile uint32_t   period;
extern bool pioirq;
extern void PIO_init();

/*-------------------------------------------------------
 * Debug and development aid tracing
 * only enabled if DEBUG is defined previously
 */

#ifdef  UART  
#define _SERIAL Serial1
#else
#define _SERIAL Serial
#endif //UART 

#ifdef CAT
#define _CAT   Serial1
#endif //CAT

#if defined(CAT) && defined(UART) && defined(DEBUG)
#undef _SERIAL
#undef _CAT
#define _CAT    Serial
#define _SERIAL Serial1
#endif //Consistency rule

//char hi[128];

#ifdef DEBUG
#define _INFOLIST(...) \
  do { \
    strcpy(hi,"@"); \
    sprintf(hi+1,__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)
//#define _INFOLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.write(hi);_SERIAL.flush();
#else //!DEBUG
#define _INFOLIST(...) (void)0

#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness
