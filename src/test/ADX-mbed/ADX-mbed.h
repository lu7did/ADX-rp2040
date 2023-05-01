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
 

/*-----------------------------------------------------
 * External references to freqPIO variables and methods
 */
extern volatile uint32_t   period;
extern bool pioirq;
extern void PIO_init();

#define _SERIAL Serial
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
