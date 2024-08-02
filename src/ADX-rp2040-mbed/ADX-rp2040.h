//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022,2024
//
// Version 3.0
// This is an experimental code porting the ADX-rp2040 code to the Arduino IDE mbed core in order 
// to implement the support for audio over USB feature
// This code relies heavily on the great work from Hitoshi, JE1RAV at the QP-7C_RP2040 transceiver
//
// Version 4.0
// Extended support for the PixiePico project (Pixie kit based FT8 28 MHz transceiver)
//
//*********************************************************************************************************
/*---------------------------------------------------------------------------------------------------*
 *                                     Program definition and parameters.                            *
 *---------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------
 * Program configuration parameters
 */
#define PixiePico               1  //Define PicoPixie board support
//#define ADX-rp2040                     1  //Define ADX board support


#define BAUD             115200
#define DEBUG                 1  //Debug mode
//#define CAT                   1  //Uncomment to activate CAT

//#define RX_SI473X             1  //Si473x receiver
//#define SUPERHETERODYNE       1  //Superhet mode (receiver frequency displaced by FREQ_BFO)

/*---------------------------------------------------------------------------------------------------*
 * Apply few integrity rules to avoid conflicting directives to make the code compilation to go wrong
 * if project PixiePico then the processor has to be rp2040Z, else a normal Raspberry Pico (rp2040)
 */

#ifdef PixiePico
#undef  RX_SI473X               //PixiePico doesn't use a Si473X chipset
#undef  SUPERHETERODYNE         //PixiePico doesn't use a SUPERHETERODYNE mode
#endif

#ifdef ADX-rp2040
#undef PixiePico                //ADX-rp2040 prevails over PixiePico
#endif 

/*---------------------------------------------------------------------------------------------------*
 * Includes and macro definitions for the ADX-rp2040 transceiver firmware                            *
 *---------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------
 * Macro expansions
 */
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0

/*---------------------------------------------------------------------------------------------------*
 *                        ADX-rp2040 board hardware definition and parameters.                            *
 *---------------------------------------------------------------------------------------------------*/
 #ifdef ADX-rp2040

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

#endif //ADX-rp2040

/*---------------------------------------------------------------------------------------------------*
 *                    PixiePico board hardware definition and parameters.                            *
 *---------------------------------------------------------------------------------------------------*/
 #ifdef PixiePico

#define FAN               14  //FAN activation line

#define LED_BRIGHT       255
#define LED_MEDIUM       127
#define LED_LOW           63
#define LED_DIMM          31

#define LED_BLINK         10

 #endif //PixiePico

/*---------------------------------------------------------------------------------------------------*
 *                    Definitions common for both PixiePico and ADX-rp2040 boards                    *
 *---------------------------------------------------------------------------------------------------*/
/*---
   Signal input pin
*/
#define FSKpin         27  //Frequency counter algorithm, signal input PIN (warning changes must also be done in freqPIO)

/*----
   Output control lines RX-TX 
*/
#define RXSW            2  //RX Switch

/*----
   Output control lines RX-TX 
*/
#define CAL            11  //Si5351 automatic calibration

/*----
   Input switch
*/
#define TXSW            8  //RX-TX Switch
/*---------------------------------------------------------------------------------------------------*
 *                                  Consistency rules and constants                            *
 *---------------------------------------------------------------------------------------------------*/

/*--------------------------
  Consistency rule: either DEBUG or CAT but not both
*/
#if defined(CAT)
#undef DEBUG
#endif //CAT

#define BANDS                 4
#define MAXBAND               9
/*-----------------------------------------------
  If RX_SI473X receiver defined declare constants
 *-----------------------------------------------*/

#ifdef RX_SI473X           //Define RESET pin for SI473x if defined

#define RESET_SI473X    1  //Si473x RESET pin
#define AM_FUNCTION     1
#define RESET_PIN       1
#define LSB             1
#define USB             2

#endif //RX_SI473X

/*-----------------------------------------------
  Frequency counting algorithm
 *-----------------------------------------------*/

#define FSK_TOUT             50 //FSK timeout in mSecs (no signal present for more than)
#define AUDIOSAMPLING     48000 //
#define FSK_MIN           20000 //Minimum frequency allowed (200 Hz)
#define FSK_MAX          300000 //Maximum frequency allowed (3000 Hz)
#define FSK_THRESHOLD         5 //Minimum threshold to change the frequency
#define FSK_MIN_CHANGE       10 //Minimum frequency change (0.10 Hz) in order to avoid small measurement errors to produce frequency shifts
#define USB_MIN               1 //Minimum USB in queue size
#define FREQ_BFO         446400 //Intermediate frequency
#define AF_TONE          150000
#define SAMPLESIZE           48


#define CAT_WARN1           500
#define CAT_WARN2         10000
#define ONE_MSEC           1000

/*------------------------------*
   ADC port values
  ------------------------------*/
#define ADC_NUM       1
#define ADC_PIN       (26 + ADC_NUM)
#define ADC_VREF      3.3
#define ADC_RANGE     (1 << 12)
#define ADC_CONVERT   (ADC_VREF / (ADC_RANGE - 1))

/*------------------------------*
   ADC Limits and operation
  ------------------------------*/
#define  ADCMAX       4096
#define  ADCMIN       0
#define  ADCZERO     (ADCMAX+ADCMIN)/2
#define  ADCSAMPLE     1 

/*------------------------------------
 DEBUG macro
*/
#ifdef DEBUG
#define _INFO(...) \
  do { \
    now = time(0) - t_ofs;  \
    gmtime_r(&now, &timeinfo);  \
    sprintf(timestr,"[%02d:%02d:%02d] ",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);   \
    strcpy(hi,timestr); \
    strcat(hi,"@");  \
    sprintf(hi+strlen(hi),"%s ",__func__); \
    sprintf(hi+strlen(hi),__VA_ARGS__); \
    Serial.write(hi); \
    Serial.flush(); \
  } while (false)
#else
#define _INFO(...) (void)0
#endif //_INFO macro definition as NOP when not in debug mode, will consume one byte of nothingness

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*                                        External references                                                    */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
extern const uint16_t Bands[BANDS];
extern uint16_t Band2Idx(uint16_t b);
extern const long unsigned slot[MAXBAND][3];
extern int Band_slot;     // This is he default starting band 1=40m, 2=30m, 3=20m, 4=10m
extern char hi[256];
extern uint64_t RF_freq;   // RF frequency (Hz)

/*------------------------------------------------------
 *   Internal clock handling
 */
extern struct tm timeinfo;        //current time
extern struct tm timeprev;        //epoch time
extern time_t t_ofs;          //time correction after sync (0 if not sync-ed)
extern time_t now;
extern char timestr[12];

/*------------------------------------------------------
 *   Optional Si473x handling
 */

#ifdef RX_SI473X
extern void SI473x_Setup();

extern bool SI473x_enabled;
extern void SI473x_setFrequency(int s);
extern int getRSSI();
extern void SI473x_Status();
extern String getFrequency();
extern int getSNR();
extern int getVolume();
extern int getSignal();
extern void setVolume();
#endif //RX_SI473X


