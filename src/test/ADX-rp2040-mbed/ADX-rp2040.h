//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022,2023
//
// Version 3.0
// This is experimental code trying to port the ADX-rp2040 code to the Arduino IDE mbed core in order 
// to implement the USB audio feature
// This code relies heavily on the great work from Hitoshi, JE1RAV at the QP-7C_RP2040 transceiver
//
//*********************************************************************************************************


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
#define BAUD            115200
#define CAT                  1  //Uncomment to activate CAT


#define FSK_TOUT             50 //FSK timeout in mSecs (no signal present for more than)
#define SAMPLE_RATE       44100 //Audio sample rate
#define FSK_MIN           20000 //Minimum frequency allowed (200 Hz)
#define FSK_MAX          300000 //Maximum frequency allowed (3000 Hz)
#define FSK_THRESHOLD         5 //Minimum threshold to change the frequency
#define FSK_MIN_CHANGE       10 //Minimum frequency change (0.10 Hz) in order to avoid small measurement errors to produce frequency shifts
#define USB_MIN               1 //Minimum USB in queue size
/*------------------------------------------------------
 *   Internal clock handling
 */
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
time_t now;
char timestr[12];


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

