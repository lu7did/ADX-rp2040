//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_mbed                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 1.0
//
// This is a PoC sketch to implement an ADX class transceiver using the mbed rp2040 core of libraries
// It's derived from the ADX-rp2040 project which in turn is a porting of the ADX_UNO firmware by 
// Barb (WB2CBA).
// This first implementation is crude, at best.....
//************************************************************************************************************************
/*-------------------------------------------------
 * Macro expansions
 */
//#define digitalWrite(x,y) gpio_put(x,y)
//#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0
/*--------------------------------------------------
 * Program configuration parameters
 */
#undef  UART

#define DEBUG                1  //Uncomment to activate debugging traces (_INFOLIST(...) statements thru _SERIAL
#define BAUD            115200
#define AUDIOSAMPLING    48000  // USB Audio sampling frequency
#define N_FREQ               2 // number of using RF frequencies　(<= 7)
#define FREQ_0         7041000 // RF frequency in Hz
#define FREQ_1         7074000 // in Hz

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0


extern uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}

/*
#define pin_RX 27 //pin for RX switch (D1,output)
#define pin_TX 28 //pin for TX switch (D2,output)
#define pin_SW 3 //pin for freq change switch (D10,input)
#define pin_RED 17 //pin for Red LED (output)
#define pin_GREEN 16 //pin for GREEN LED (output)
#define pin_BLUE 25 //pin for BLUE LED (output) 
#define pin_LED_POWER 11 //pin for NEOPIXEL LED power (output)
#define pin_LED 12 //pin for NEOPIXEL LED (output)
*/



// =================================================

#define Si5351_CAL 0 // Calibrate for your Si5351 module.
#define FREQ_BFO 446400 // in Hz: Calibrate for your Filter.

/*---------------------------------------------------------------------------------------------
                                 PORTING DEFINES
   The following defines are used for porting and testing purposes
  ---------------------------------------------------------------------------------------------*/
#define mbed    1

/*---------------------
 * Definitions related to the autocalibration function
 */
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
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
    I2C
*/
#define I2C_SDA        16  //I2C SDA
#define I2C_SCL        17  //I2C SCL

/*----
   Define UART0 (GPIO 12/pin 16)
*/
#define UART_TX        12
#define UART_RX        13

#define SI5351_REF 		25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz


/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
//extern volatile uint32_t   period;
//extern bool pioirq;
//extern void PIO_init();

#define _SERIAL Serial
#define _CAT uartSerial

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
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)
#else //!DEBUG
#define _INFO(...) (void)0

#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness
