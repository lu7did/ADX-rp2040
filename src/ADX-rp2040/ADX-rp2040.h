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
//#define UART                 1  //Uncomment to use UART0 as the main output
#define BAUD            115200
#define CAT                  1  //Uncomment to activate CAT
 
/*-----------------------------------------------------
   Definition of CAT (comment out if not desired)
*/

/*--------------------------------------------------
   Program configuration parameters
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
