//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 1.0
// 
// This is a direct port into the rp2040 architecture of the ADX_UNO firmware code (baseline version 1.1).
// (add) Apr,28th Added CAT functionality
//
// Version 2.0
// Addition of the ddsPIO feature where the Si5351 chip is replaced by firmware at the rp2040 providing
// both CLK0 & CLK1 of the board
// Overall clean-up of the code to remove non-rp2040 code
//
//*********************************************************************************************************
// FW VERSION: ADX_UNO_V1.1 - Version release date: 08/05/2022
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//  Version History
//  V1.0  Modified Calibration EEPROM to protect R/W cycle of EEPROM.
//  V1.1 10m/28Mhz band support added.
//*********************************************************************************************************
//
// Required Libraries and build chain components
//
// Created with Arduino IDE using the Arduino-Pico core created by Earle F. Philhower, III available
// at https://github.com/earlephilhower
// Check for installation and configuration instructions at https://www.upesy.com/blogs/tutorials/install-raspberry-pi-pico-on-arduino-ide-software
//
//                                     *******************************************
//                                     *                Warning                  *
//                                     *******************************************
//
// This firmware is meant to be used with an ADX board where the Arduino Nano or Arduino Uno processor has been replaced
// by a raspberry pi pico board plus addicional voltage and signal conditionin circuits, please see the host site
// https://github.com/lu7did/ADX-rp2040 for construction details and further comments.
//
// This firmware is meant to be compiled using the latest Arduino IDE environment with the following parameters
//
// Board: "Raspberry Pi Pico"
// Flash size: "2 Mb (no FS)
// CPU Speed: 133 MHz
// Optimize: Small -Os (Standard)
// RTTi: disabled
// Stack protector: disabled
// C++ Exceptions: disabled
// Debug port: disabled
// Debug level: none
// USB stack: "Adafruit TinyUSB"
// IP Stack: "IPv4 only"
//
// The firmware has not been tested with a Raspberry Pi Pico W version
//*----------------------------------------------------------------------------------------------------------------------
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//*----------------------------------------------------------------------------------------------------------------------
// if !ddsPIO (uses a Si5351 to generate the clock)
//
//    Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide)
//    SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//    IMPORTANT NOTE: Use V2.1.3 of NT7S SI5351 Library. This is the only version compatible with ADX!!!*
//    Arduino "Wire.h" I2C library(built-into arduino ide)
//    SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//    *********************************************************************************************
//    *                        SI5351 VFO CALIBRATION PROCEDURE                                   *
//    *********************************************************************************************
//    For SI5351 VFO Calibration Procedure follow these steps:
//    1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope
//        that can measure 1 Mhz up to 1Hz accurately.
//    2 - Press SW2 / --->(CAL) pushbutton and hold.
//    3 - Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton.
//    4 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL).
//        Now Calibration mode is active.
//    5 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
//    6 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope.
//        The waveform is Square wave so freqency calculation can be performed esaily.
//    7 - If you read as accurate as possible 1000000 Hz then calibration is done.
//    8 - Now we must save this calibration value to EEPROM location.
//        In order to save calibration value, press TX button briefly. TX LED will flash 3
//        times which indicates that Calibration value is saved.
//    9 - Power off ADX.
//*----------------------------------------------------------------------------------------------------------------------
//    *********************************************************************************************
//    *                        CAT CONTROL SETTINGS AND CAT FUNCTIONALITY                         *
//    *********************************************************************************************
//    Extract from ADX_CAT V1.4 header
//    CAT CONTROL RIG EMULATION: KENWOOD TS2000
//    SERIAL PORT SETTINGS: 115200 baud,8 bit,1 stop bit
//    When CAT is active FT4 and JS8 leds will be solid lit.
//         In CAT mode none of the Switches and leds are active including TX SWITCH in order to avoid different setting clashes except TX LED. 
//         TX LED WILL BE LIT briefly on/off and then solid during TX WHEN TRANSMITTING IN CAT Mode.
//    In CAT mode ADX can be controlled ONLY by CAT interface. Frequency and TX can be controlled via CAT.
//         To get out of CAT mode and to use ADX with Switch and led interface just recycle power. Once activated CAT mode stays active as rig control Until power recycle. 
//    In CAT mode manual Band setup is deactivated and ADX can be operated on any band as long as the right lpf filter module is plugged in. 
//    In CAT MODE MAKE SURE THE CORRECT LPF MODULE IS PLUGGED IN WHEN OPERATING BAND IS CHANGED!!! IF WRONG LPF FILTER MODULE IS PLUGGED IN then PA POWER MOSFETS CAN BE DAMAGED
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//    *********************************************************************************************
//    *                        Libraries being used                                               *
//    *********************************************************************************************

#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
/*---------------------------------------------------------------------------------------------
                                 PORTING DEFINES
   The following defines are used for porting and testing purposes
  ---------------------------------------------------------------------------------------------*/

/*-----------------------------------------------
   Pico Arduino CORE includes
*/
#include <stdint.h>
#include "hardware/watchdog.h"
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
//#include <WiFi.h>
#include <Time.h>

char hi[128];

#include "ADX-rp2040.h"
/*-----------------------------------------------------
   External references to freqPIO variables and methods
*/
extern volatile uint32_t   period;
extern bool pioirq;
extern void PIO_init();

/*-----------------------------------------------------
   External references to ddsPIO variables and methods
*/

extern double fpio;
extern void ddsPIO_configure();
extern void ddsPIO_setFrequency(double f);

//    *********************************************************************************************
//    *                        Variable definitions                                               *
//    *********************************************************************************************

uint32_t codefreq = 0;
uint32_t prevfreq = 0;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
                                   End of porting definitions
  =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//*********************************************************************************************
//*                        Variable definitions (ADX original)                                *
//*********************************************************************************************

int      temp;
int      addr = 0;
int      mode;
int      TX_State = 0;
int      UP_State;
int      DOWN_State;
int      TXSW_State;
int      Bdly = 250;
int      cat_stat = 0;
int      CAT_mode = 2;   

uint32_t val_EE;
uint32_t val;
int32_t  cal_factor;

unsigned long freq;
unsigned long freq1;


/*
unsigned long F_FT8;
unsigned long F_FT4;
unsigned long F_JS8;
unsigned long F_WSPR;
*/
#ifdef CAT
/*-------------------------------------------------------------------------------------------
  CAT definitions
*/
#define CATCMD_SIZE          64

char buf[CATCMD_SIZE];
char CATResp[CATCMD_SIZE];
char CATCmd[CATCMD_SIZE];
char serialBuf[CATCMD_SIZE*2];
char resp[CATCMD_SIZE*2];
bool ignoreFA=false;

int  CATT1=0;
int  CATT2=0;
bool flipLED=false;

#endif //CAT

/*---------------------
 * Definitions related to the autocalibration function
 */
#ifdef AUTOCAL 
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
int      pwm_slice;
uint32_t f_hi;
uint32_t fclk     = 0;
int32_t  error    = 0;
#endif //AUTOCAL

//**********************************[ BAND SELECT ]************************************************
// ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4
// from supported 8 bands.
// To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay
// lit for the stored band. also TX LED will be lit to indicate
// that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When
// desired band bank is selected press TX button briefly to exit band select mode.
// Now the new selected band bank will flash 3 times and then stored mode LED will be lit.
// TX won't activate when changing bands so don't worry on pressing TX button when changing bands in
// band mode.
// Assign your prefered bands to B1,B2,B3 and B4
// Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m, 10m
int slot[4]   = {40,30,20,10};
int Band_slot = SLOT;                  //This is the default band Band1=1,Band2=2,Band3=3,Band4=4
int Band      =   10;                  //This is the default band 
           
long unsigned int Bands[NBANDS][NMODES] = {
              { 3568600, 3578000, 3575000, 3573000},
              { 7038600, 7078000, 7047500, 7074000},
              {10138700,10130000,10140000,10136000},
              {14095600,14078000,14080000,14074000},
              {18104600,18104000,18104000,18100000},
              {21094600,21078000,21140000,21074000},
              {28124600,28078000,28180000,28074000}};

#ifndef DDSPIO
Si5351 si5351;
#endif //!DDSPIO


//*************************************[ SETUP FUNCTION ]**************************************
void setup()
{
/*------------------------------------
  Setup default LED
*/

//pinMode(LED_BUILTIN, OUTPUT);
//gpio_set_mask(1 << LED_BUILTIN);

/*------------------------------------
  Setup Serial USB Monitor line
*/

_SERIAL.begin(115200);
_SERIAL.setTimeout(4);


#ifdef DEBUG
while (!_SERIAL); 
  //gpio_set_mask(1<<LED_BUILTIN);
  //delay(250);
  //gpio_clr_mask(1<<LED_BUILTIN);
  //delay(250);

#endif //DEBUG

  delay(50);
  _SERIAL.flush();

  _INFOLIST("%s Firmware %s version(%s) build(%s)\n",__func__,PROGNAME,VERSION,BUILD);
#ifdef CAT

  _CAT.setTX(UART_TX);
  _CAT.setRX(UART_RX);

  cat_stat = 0;
  _CAT.begin(115200);
  delay(50);
  _CAT.flush();
//_CAT.setTimeout(4);

  strcpy(CATResp,"");
  strcpy(CATCmd,"");
  strcpy(serialBuf,"");
  strcpy(resp,"");
  strcpy(buf,"");

#endif //CAT

  /*-----------------------------
     Port definitions (pinout, direction and pullups used
  */
  /*--------
     Initialize switches
  */
  gpio_init(UP);
  gpio_init(DOWN);
  gpio_init(TXSW);

  /*----
     Initialize RX command
  */
  gpio_init(RX);

  /*---
     Initialize LED
  */

  gpio_init(WSPR);
  gpio_init(JS8);
  gpio_init(FT4);
  gpio_init(FT8);
  gpio_init(TX);

  /*-----
     Set direction of input ports
  */
  gpio_set_dir(UP, GPIO_IN);
  gpio_set_dir(DOWN, GPIO_IN);
  gpio_set_dir(TXSW, GPIO_IN);

  /*-----
     Pull-up for input ports
     (Warning! See hardware instructions
  */

  gpio_pull_up(TXSW);
  gpio_pull_up(DOWN);
  gpio_pull_up(UP);

  /*---
     Set output ports
  */
  gpio_set_dir(RX, GPIO_OUT);
  gpio_set_dir(TX, GPIO_OUT);
  gpio_set_dir(WSPR, GPIO_OUT);
  gpio_set_dir(JS8, GPIO_OUT);
  gpio_set_dir(FT4, GPIO_OUT);
  gpio_set_dir(FT8, GPIO_OUT);

  /*----
     Digital input pin, it's an ADC port to allow further development of DSP based inputs
  */

  gpio_init(FSKpin);
  gpio_set_dir(FSKpin, GPIO_IN);
  gpio_pull_up(FSKpin);

  /*---
     Initialice I2C sub-system only if a Si5351 chip will be used
  */

#ifndef DDSPIO
  
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

#endif //!DDSPIO

  _INFOLIST("%s: I/O setup completed\n", __func__);
  /*-----------
       System initialization
  */
  INIT();

#ifndef DDSPIO
  //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  //----------------------------------------------------------------------------------
  bool rc=si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  
  if(!rc) {
    _INFOLIST("%s Device not found on I2C bus!\n",__func__);
  } else {
    _INFOLIST("%s Device Si5351 found on I2C bus\n",__func__);
  }

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock
  _INFOLIST("%s si5351 clock initialization completed\n", __func__);

#endif //!DDSPIO

  if ( digitalRead(DOWN) == LOW ) {

     #ifdef AUTOCAL
        _INFOLIST("%s Automatic calibration mode started\n", __func__);
        AutoCalibration();
        _INFOLIST("%s Calibration mode started\n", __func__);
        Calibration();
     #endif //AUTOCAL
     
  }

  /*--------------------------------------------------------------------------
    PIO counting algorithm definition
  */
  PIO_init();
  _INFOLIST("%s PIO sub-system initialized\n", __func__);



  
  /*--------------------------------------------------------------------------
    ddsPIO enabled, setup and configure
  */
#ifdef DDSPIO
/*------------------------------------
  Configure PIO and set initial frequency
*/

  fpio=freq;
  ddsPIO_configure();
//  ddsPIO_setFrequency(fpio);
  _INFOLIST("%s DDSPIO sub-system initialized\n",__func__);

#endif //DDSPIO 

/*--------------------
     Assign initial band and mode
  */
  Band_assign();
  //Mode_assign();

  /*--------------------
     Place the receiver in reception mode
  */
  digitalWrite(RX, HIGH);
  _INFOLIST("%s setup completed successfully\n", __func__);

}

//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{


/*------------------------------------------------
     Check serial port for new CAT frames and 
     respond to them.
  */

  #ifdef CAT

  CAT_check();
  
  #endif //CAT
  /*------------------------------------------------
     Explore and handle interactions with the user
     thru the UP/DOWN or TX buttons
  */
  UP_State = digitalRead(UP);
  DOWN_State = digitalRead(DOWN);

  /*----
     UP(Pressed) && DOWN(Pressed) && !Transmitting
     Start band selection mode
  */

  if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0) && (cat_stat == 0 )) {
    delay(100);
    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);
    if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0) && (cat_stat == 0)) {
      Band_Select();
    }
  }

  /*----
     UP(Pressed) && DOWN(!Pressed) and !Transmitting
     Increase mode in direct sequence
  */

  if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0) && (cat_stat == 0)) {
    delay(100);
    UP_State = digitalRead(UP);
    if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0) && (cat_stat == 0)) {
      mode = mode - 1;
      if (mode < 1) {
        mode = 4;
      }

      addr = 40;                   //Save current mode in EEPROM
      EEPROM.put(addr, mode);
      EEPROM.commit();   //rp2040 doesn't have any EEPROM, the core library emulates EEPROM on flash memory, but it requires a commit() to set
      Mode_assign();
    }
  }

  /*----
     UP(!Pressed) && DOWN(Pressed) && !Transmitting
     Change mode in the opposite sequence

  */
  DOWN_State = digitalRead(DOWN);
  if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0) && (cat_stat == 0)) {
    delay(50);

    DOWN_State = digitalRead(DOWN);
    if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0) && (cat_stat == 0)) {
      mode = mode + 1;

      if (mode > 4) {
        mode = 1;
      }

      addr = 40;
      EEPROM.put(addr, mode);

      EEPROM.commit(); //rp2040 doesn't have an EEPROM, the core library emulates EEPROM on flash memory and requires a commit() to set values permanently
      Mode_assign();

    }
  }

  /*----
     If the TX button is pressed then activate the transmitter until the button is released
  */
  TXSW_State = digitalRead(TXSW);

  if ((TXSW_State == LOW) && (TX_State == 0)) {
    delay(50);

    TXSW_State = digitalRead(TXSW);
    if ((TXSW_State == LOW) && (TX_State == 0)) {
      Mode_assign();
      ManualTX();
    }
  }

  /*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
     This is the audio frequency counting algorithm to be used when operating under the rp2040 architecture
     A microkernel implemented on a PIO runs an edge detector on pin FSKpin raising an interrupt when it happens,
     the elapsed between sucessive interrupts is then measured and used to establish the period between them,
     which in turn is used to compute the frequency. Then VOX mechanism allows for 15 msecs (VOX_MAXTRY * 1 uSec)
     before to turn the TX off allowing some tolerance for noise and transients.
     The frequency measured is processed with a pseudo-pass band [FSKMIN,FSKMAX] to remove transient and other
     noises, the algorithm will recover in the next counting cycle.
     The entire code segment is nullified when NOT compiling with the RP2040 configuration
     The interrupt handler is defined in the freqPIO.cpp module and the PIO RISC ASM counting in freqPIO.pio.h
    =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
  int FSK = VOX_MAXTRY;
  int FSKtx = 0;

  int k = 0;

  while ( FSK > 0 ) {                                //Iterate up to MAX_TRY times looking for signal to transmit
    if (pioirq == true) {                            //The interrupt handler produced a new period value
      pioirq = false;                                //clear the condition to allow the next to happen
      FSK = VOX_MAXTRY;                              //restore the "silence counter", a valid frequency reading extends it
      if (period > 0) {                              //If the period is above zero compute the frequency
        codefreq = FSK_USEC / period;
      } else {
        codefreq = 0;
      }
      /*------------------------------------------------------*
        Filter out frequencies outside the allowed bandwidth
        ------------------------------------------------------*/
      if (codefreq >= uint32_t(FSKMIN) && codefreq <= uint32_t(FSKMAX)) {
        FSK = VOX_MAXTRY;
        /*----------------------------------------------------*
          if VOX is off then pass into TX mode
          Frequency IS NOT changed on the first sample
          ----------------------------------------------------*/
        if (FSKtx == 0) {
          setTX(HIGH);
          FSKtx = 1;
          prevfreq = 0;
          FSK = VOX_MAXTRY;
          _INFOLIST("%s VOX turned ON\n", __func__);
          continue;
        }
        /*-----------------------------------------------------
           Avoid producing jitter by changing the frequency
           by less than 4 Hz.
        */
        if (abs(int(codefreq - prevfreq)) >= FSK_ERROR) {
          unsigned long fdds = ((unsigned long)(freq + codefreq));

          #ifdef DDSPIO
             _INFOLIST("%s freq=%ld codefreq=%ld ddsPIO(f)=%lu\n", __func__, freq, codefreq, fdds);
             ddsPIO_setFrequency(fdds);
          #else   
             _INFOLIST("%s freq=%ld codefreq=%ld si5351(f)=%lu\n", __func__, freq, codefreq, fdds);
             si5351.set_freq(fdds * 100ULL, SI5351_CLK0);
          #endif //DDSPIO   

          prevfreq = codefreq;
          FSK = VOX_MAXTRY;
        }
      }
    } else {
      /*--------------------
        Waiting for signal, rp2040 is way faster than ATMEGA328p thus a delay is needed
        --------------------*/
      uint32_t tcnt = time_us_32() + uint32_t(FSK_IDLE);
      while (time_us_32() < tcnt);
      
      if (FSK > 0) {
          FSK--;
      }

      if (FSK == 0 && TX_State == 1 && cat_stat == 0) {
        _INFOLIST("%s VOX turned OFF\n", __func__);
      }
    }
#ifdef CAT  
    CAT_check();
    CAT_warning();
#endif //CAT

  }
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

  /*---------
     If the code fallbacks here it's because there is no audio input anymore
     then switch the transmitter off and set it to receive
  */
  if (TX_State == 1 && cat_stat == 0) {
      setTX(LOW);
  }

  #ifdef CAT  
    CAT_check(); 
  #endif //CAT
}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of loop()                                                      *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                       Support functions                                                    *
//                       (mostly original from ADX_UnO except few debug messages)                             *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

#ifdef CAT
/*-------------------------------------
  Trim a string
*/
char *trim(char *str)
{
  char *end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator character
  end[1] = '\0';

  return str;
}
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                     CAT Support functions                                                  *
//                partial implementation of the Kenwood TS2000 protocol (115200 8N2).                         *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void CAT_warning() {

      if (millis()-CATT1 >=500 && CATT2 !=0 && cat_stat != 0) {

       flipLED=!flipLED;
       digitalWrite(WSPR, flipLED); 
       digitalWrite(FT8, flipLED);
       CATT1=millis();

       if (millis()-CATT2 >= 10000) {

          CATT1=0;
          CATT2=0;
          digitalWrite(WSPR, LOW); 
          digitalWrite(FT8, LOW);

       } 

      }
}      
/*------------------------------
  CAT_process
  receives a string with a CAT command (delimited by "";"") and reacts with the
  proper answer to that command.
  Only FA,IF,TX and RX commands are properly implemented, the rest of the commands produces
  mocked up responses to satisfy the logic of (mostly) WSJT-X
*/
bool CAT_process(char *c,char *r,char *cmd,char *arg){

  char *q;
  strcpy(cmd,"");
  strcpy(arg,"");

  cmd[0]=c[0];
  cmd[1]=c[1];
  cmd[2]=0x00;

  if (strlen(cmd) != strlen(c)) {
     strcpy(arg,&c[2]);
  } else {
     strcpy(arg,"");
  }

  if (strlen(cmd)<2) {
    _INFOLIST("%s malformed command, ignored\n",__func__);
    return false;
  }

//* ERROR 
//* wrong definition of fx 
  if (strcmp(cmd,"FA")==0) {  
      if (strcmp(arg,"") != 0) {
        unsigned long fdds=strtol(arg, &q, 10);
        freq=fdds;        
        freq1=fdds;
        #ifdef DDSPIO
           ddsPIO_setFrequency(freq);
        #else
           si5351.set_freq(freq * 100ULL, SI5351_CLK0);
           si5351.set_freq(freq * 100ULL, SI5351_CLK1);
        #endif //DDSPIO
        CATT1=millis();
        CATT2=millis();
        setTX(LOW);
      }    
      String sent = "FA" // Return 11 digit frequency in Hz.  
          + String("00000000000").substring(0,11-(String(freq).length()))   
          + String(freq) + ";";     
      strcpy(r,sent.c_str());
      return true;
  }
  

  if (strcmp(cmd,"PS")==0) {
      strcpy(r,"PS1;");
      return true;
  }

  if (strcmp(cmd,"TX")==0)  {   
      strcpy(r,"TX0;");
      setTX(HIGH);    
      return true;
  } 

  if (strcmp(cmd,"RX")==0) {  
    strcpy(r,"RX0;");
    setTX(LOW);
    return true;       
  }

  if (strcmp(cmd,"ID")==0) {  
      strcpy(r,"ID019;");
      return true;
  }

  if (strcmp(cmd,"AI")==0) {
      strcpy(r,"AI0;"); 
      return true;
  }

//*---

  if (strcmp(cmd,"IF")==0) {

      if (TX_State == 1) {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String((long int)freq).length()))   
                  + String((long int)freq) + "0000" + "+" + "00000" + "0" + "0" + "0" + "00" + "12" + "0000000;"; 
          strcpy(r,sent.c_str());        
      } else {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String((long int)freq).length()))   
                  + String((long int)freq) + "0000" + "+" + "00000" + "0" + "0" + "0" + "00" + "02" + "0000000;"; 
          strcpy(r,sent.c_str());
      } 
      return true;
  }

//*---

  if (strcmp(cmd,"MD")==0) {  
      strcpy(r,"MD2;");
      return true;
  }
  strcpy(r,"ID019;");   //dummy answer trying not to disrupt the CAT protocol flow
  _INFOLIST("%s ***ERROR*** Entry(%s) nor processed response(%s)\n",__func__,c,r);
  return false;

}

/*-----------------------------------------------
  CAT_check
  Read the serial port and parses the input for
  non-confirming structures
  parse the input stream and tokenize the commands
  found on it.
  Take into consideration some protocol deviations
  (oddities) required by WSJT-X (or HamLib, I don't know)
  to properly operate
*/
void CAT_check(void) {

bool flagRXTX=false;
char cmd[4];
char arg[16];

/*----------
  Handle the brief frequency change
*/
CAT_warning();

/*----------
  Check if data is available on the serial port
*/
int nread=_CAT.available();
if (nread > 0){
   if (cat_stat == 0) {
       cat_stat=1;
       digitalWrite(WSPR, LOW); 
       digitalWrite(FT8, LOW);
       digitalWrite(JS8, HIGH);
       digitalWrite(FT4, HIGH);
       setTX(LOW);
   }
} else {
  return;
}

/*-----------
  Too small as a packet, perhaps fragmentation is occuring
*/
if (nread < 3) { return; }

/*-----------
  Read the serial port buffer
*/
int rc=_CAT.readBytes(buf,nread);
if (rc <= 1) {return;}
buf[rc]=0x0;

/*------------
  Look after spurious contents
*/
int k=strlen(serialBuf);
for (int i=0;i<strlen(buf);i++){
    char c=buf[i];
    if (c>= 0x20 && c<=0x5f && c!=0x0a && c!=0x0d) {
       serialBuf[k++]=c;
    }
}
serialBuf[k]=0x00;
    
/*-------------
  Fragmentation might occur
*/
if (serialBuf[strlen(serialBuf)-1] != ';') {
   return;
}

/*--------------
  Look for oddities from WSJT-X, if this string is
  received the action to turn the TX on is expected
  but only the answer to the ID command needs to 
  be sent
*/
_INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,serialBuf,strlen(serialBuf));

if (strcmp(serialBuf,"TX;ID;") == 0) {

    setTX(HIGH);
    strcpy(CATResp,"ID019;");
    _CAT.print(CATResp);
    _INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,CATResp,strlen(CATResp));

    strcpy(CATResp,"");
    strcpy(serialBuf,"");      
    return;

}

/*-------------------
  More oddities, now with the receiving part
*/
    if (strcmp(serialBuf,"RX;ID;") == 0) {      
       setTX(LOW);
       strcpy(CATResp,"ID019;");
       _CAT.print(CATResp);
       _INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,CATResp,strlen(CATResp));

       strcpy(CATResp,"");
       strcpy(serialBuf,"");      
       return;

    }

/*----------------------
  Parse the command using the ";" as the
  token delimiter
*/
    int j=0;
    strcpy(CATCmd,"");
    strcpy(resp,"");
    int last=0;

    for(int i=0;i<strlen(serialBuf);i++) {
       char data=serialBuf[i];
       if (data==';') {
          last=i;
          strcpy(cmd,"");
          strcpy(arg,"");         

          /* EOT mark found --> process CAT command */
          
          if (!CAT_process(CATCmd,CATResp,cmd,arg)) {
            _CAT.print(CATResp);
            _INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,CATResp,strlen(CATResp));
             strcpy(serialBuf,"");
             strcpy(CATCmd,"");
             strcpy(CATResp,"");
             return;
          }

          /*--- Yet another WSJT-X oddity ---*/

          if (strcmp(cmd,"FA")==0) {
             if (ignoreFA==true) {
                ignoreFA=false;
                strcpy(CATCmd,"");
                strcpy(CATResp,"");
             } else {
                _CAT.print(CATResp);
                _INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,CATResp,strlen(CATResp));
                strcpy(resp,"");
                ignoreFA=true;
             }   
          } else {
             strcat(resp,CATResp);
             ignoreFA=false;
          }   
          strcpy(CATCmd,"");
          strcpy(CATResp,"");
          j=0;
       } else {

         /*--- Between tokens store the incoming data */
         CATCmd[j++]= data;
         CATCmd[j]=0x00;
       }
    }

    /*------
     Decide whether fragmentation happened 
     */
    if (last != strlen(serialBuf)) {
       strcpy(serialBuf,&serialBuf[last+1]);
    } else {
       strcpy(serialBuf,"");
    }       

    /*-------
     Reply to any pending message
     */
    if (strlen(resp)>0) {
       _CAT.print(resp);      
       _INFOLIST("%s CAT Command(%s) len(%d)\n",__func__,resp,strlen(resp));

       if (strcmp(cmd,"RX")==0 || strcmp(cmd,"TX") == 0 ) delay(50);
    }

    /*-------- 
      Clean up buffers
    */
    strcpy(resp,"");
    strcpy(CATCmd,"");
    strcpy(CATResp,"");
    return;
}
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of CAT Protocol handler                                        *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#endif //CAT

/*--------------------------------------------
  Convert a band in meters to the index into the Bands table [0..6]
*/  
int band2idx(int b){

int i=0;

switch(b) {
  case 80: i=0; break;
  case 40: i=1; break;
  case 30: i=2; break;
  case 20: i=3; break;
  case 17: i=4; break;
  case 15: i=5; break;
  case 10: i=6; break;

  default:
    i=6; break;
}
_INFOLIST("%s band(%d) idx(%d)\n",__func__,b,i);
return i;

}

//************************************[ MODE Assign ]**********************************
/*-------------------------
  Assign mode and set the configuration based on that
*/

void Mode_assign() {

  addr = 40;
  EEPROM.get(addr, mode);
  _INFOLIST("%s EEPROM got mode(%d) previously Band(%d)\n",__func__,mode,Band);
  int b=band2idx(Band);
  freq=Bands[b][mode-1];
  #ifdef DDSPIO
    fpio=freq;
    _INFOLIST("%s ddsPIO(f)=%lu\n", __func__, freq);
    ddsPIO_setFrequency(freq);
  #else   
    _INFOLIST("%s si5351(f)=%lu\n", __func__, freq);
    si5351.set_freq(freq * 100ULL, SI5351_CLK0);
  #endif //DDSPIO   
  clearLED();
  switch(mode) {
     case 1: setLED(WSPR,HIGH);break;
     case 2: setLED(JS8,HIGH);break;
     case 3: setLED(FT4,HIGH);break;
     case 4: setLED(FT8,HIGH);break;
  }
  _INFOLIST("%s mode(%d) Band(%d) index(%d) freq(%ld)\n",__func__,mode,Band,b,freq);

}

//********************************[ END OF MODE ASSIGN ]*******************************

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign() {

  _INFOLIST("%s mode=%d freq=%ld\n", __func__, mode, freq);

}
//************************[ End of Frequency assign function ]*************************

//******************************[ Band  Assign Function ]******************************
/*-------------------------
  Set a given LED
*/
void setLED(int LED,int state) {
  digitalWrite(LED,state);
  //_INFOLIST("%s LED(%d) state(%d)\n",__func__,LED,state);
}
/*------------------------
  Clear all LED
*/
void clearLED() {
  for (int i=FT8;i<WSPR+1;i++){
      setLED(i,LOW);
  }
  //_INFOLIST("%s Clear LED[*]\n",__func__);
}
/*-----------------------
  Blink selected LED 4 times
*/
void blinkLED(int LED){
  for (int i=0;i<4;i++) {
    setLED(LED,HIGH);
    delay(Bdly);
    setLED(LED,LOW);
    delay(Bdly);
  }
  _INFOLIST("%s Flashing LED[%d]\n",__func__,LED);
}
/*---------------
  Convert slot 2 band
*/
int slot2Band(int s) {
  if (s < 1 || s > 4) {
    s = 4;
  }
  int b=slot[s-1];
  _INFOLIST("%s Slot(%d) --> Band(%d)\n",__func__,s,b);
  return b;
}
/*----------------------------------------------------------
  Assign band based on defaults or EEPROM saved data
*/
void Band_assign() {

  clearLED();
  addr = 50;
  EEPROM.get(addr, Band_slot);
  Band=slot2Band(Band_slot);
  blinkLED(Band_slot+3);

  delay(1000);
  Mode_assign();

  _INFOLIST("%s band_slot=%d mode=%d band=%d\n", __func__, Band_slot, mode, Band);
}
//***************************[ End of Band assign function ]***************************


//*******************************[ Manual TX FUNCTION ]********************************
void setTX(bool tx) {
   if (tx) {

      TX_State = 1;
      
      digitalWrite(RX, LOW);
      digitalWrite(TX, HIGH);

      #ifdef DDSPIO
    fpio=freq;
         ddsPIO_setFrequency(freq);
      #else 
         si5351.set_freq(freq1 * 100ULL, SI5351_CLK0);
         si5351.output_enable(SI5351_CLK1, 0);   //RX off
         si5351.output_enable(SI5351_CLK0, 1);   // TX on
      #endif //!DDSPIO

      _INFOLIST("%s TX+\n",__func__);

   } else {

      digitalWrite(TX, LOW);
      
      #ifndef DDSPIO
        si5351.output_enable(SI5351_CLK0, 0);   //TX off
        si5351.set_freq(freq * 100ULL, SI5351_CLK1);
        si5351.output_enable(SI5351_CLK1, 1);   //RX on
      #endif //!DDSPIO

      TX_State = 0;
      digitalWrite(RX, HIGH);
      _INFOLIST("%s TX-\n",__func__);
  
   }
}
void ManualTX() {
  setTX(HIGH);

TXON:

  TXSW_State = digitalRead(TXSW);
  if (TXSW_State == HIGH) {
    goto EXIT_TX;

  }
  goto TXON;

EXIT_TX:
  setTX(LOW);
}

//********************************[ END OF Manual TX ]*********************************

//******************************[ BAND SELECT Function]********************************
void Band_Select() {


  digitalWrite(TX, 1);

  addr = 50;
  EEPROM.get(addr, Band_slot);
  clearLED();
  blinkLED(Band_slot+3);


Band_cont:
 
  clearLED();
  setLED(Band_slot+3,HIGH);
  
  UP_State = digitalRead(UP);
  DOWN_State = digitalRead(DOWN);

  if ((UP_State == LOW) && (DOWN_State == HIGH)) {
    delay(100);

    UP_State = digitalRead(UP);
    if ((UP_State == LOW) && (DOWN_State == HIGH)) {
      Band_slot = Band_slot - 1;

      if (Band_slot < 1) {
        Band_slot = 4;
      }
      _INFOLIST("%s <UP> Band_slot=%d\n", __func__, Band_slot);

    }
  }

  if ((UP_State == HIGH) && (DOWN_State == LOW)) {
    delay(100);

    DOWN_State = digitalRead(DOWN);
    if ((UP_State == HIGH) && (DOWN_State == LOW)) {
      Band_slot = Band_slot + 1;

      if (Band_slot > 4) {
        Band_slot = 1;
      }
      _INFOLIST("%s <DOWN> Band_slot=%d\n", __func__, Band_slot);

    }
  }


  TX_State = digitalRead(TXSW);
  if (TX_State == LOW) {
    delay(100);

    TX_State = digitalRead(TXSW);
    if (TX_State == LOW) {
      digitalWrite(TX, 0);
      goto Band_exit;

    }
  }

  goto Band_cont;

Band_exit:

  addr = 50;
  EEPROM.put(addr, Band_slot);
  EEPROM.commit();
  Band_assign();
  _INFOLIST("%s completed set Band_slot=%d\n", __func__, Band_slot);


}

//*********************************[ END OF BAND SELECT ]*****************************
/*----------------------
 * Interrupt handler to perform a frequency counting used in calibration
 */
void pwm_int() {
  pwm_clear_irq(pwm_slice);
  f_hi++;
}

void setCalibrationLED(uint16_t e) {

  if (e>75) {
     setLED(WSPR,HIGH);
     setLED(JS8,HIGH);
     setLED(FT4,HIGH);
     setLED(FT8,HIGH);
     return;
  }
  if (e>50) {
     setLED(WSPR,HIGH);
     setLED(JS8,HIGH);
     setLED(FT4,HIGH);
     setLED(FT8,LOW);
     return;
  }
  if (e>25) {
     setLED(WSPR,HIGH);
     setLED(JS8,HIGH);
     setLED(FT4,LOW);
     setLED(FT8,LOW);
     return;
  }

  if (e>10) {
     setLED(WSPR,HIGH);
     setLED(JS8,LOW);
     setLED(FT4,LOW);
     setLED(FT8,LOW);
     return;
  }
  setLED(WSPR,LOW);
  setLED(JS8,LOW);
  setLED(FT4,LOW);
  setLED(FT8,LOW);
  return;

}
//***************************[SI5351 VFO Auto-Calibration Function]********************
//* This function has no equivalent on the ADX-UnO firmware and can only be activated
//* with the RDX or ADX2RDX boards
//*
//* To enable uncomment the #define AUTOCAL     1 statement
//*************************************************************************************
void AutoCalibration () {

#ifdef DDSPIO    //Calibration isn't defined for ddsPIO  
  _INFOLIST("%s Function not available (or needed) with ddsPIO\n",__func__);
#else
bool b = false;

  if (!Serial) {
     Serial.begin(115200);
     Serial.flush();
  }
  sprintf(hi,"Autocalibration procedure started\n");
  Serial.print(hi);

  sprintf(hi,"Current cal_factor=%d\n",cal_factor);
  Serial.print(hi);

  addr = 10;
  EEPROM.get(addr, cal_factor);

  sprintf(hi,"Current cal_factor=%d, reset\n",cal_factor);
  Serial.print(hi);
  
  cal_factor=0;
  EEPROM.put(addr, cal_factor);
  EEPROM.commit();

  digitalWrite(TX,LOW);
  setCalibrationLED(1000);
  
  while (!digitalRead(DOWN));
  /*--------------------

  */
  gpio_init(CAL);
  gpio_pull_up(CAL);
  gpio_set_dir(CAL, GPIO_IN);
  delay(10);

  /*----
    Prepare Si5351 CLK2 for calibration process
    ---*/

  gpio_set_function(CAL, GPIO_FUNC_PWM); // GP9
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Enable the clock for calibration
  si5351.set_clock_pwr(SI5351_CLK1, 0); // Enable the clock for calibration
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(Cal_freq * 100UL, SI5351_CLK2);

  sprintf(hi,"Si5351 clock setup f %lu MHz\n",(unsigned long)Cal_freq);
  Serial.print(hi);
  
  /*--------------------------------------------*
     PWM counter used for automatic calibration
     -------------------------------------------*/
  fclk = 0;
  int16_t n = int16_t(CAL_COMMIT);
  cal_factor = 0;

  pwm_slice = pwm_gpio_to_slice_num(CAL);

  /*---------------------------------------------*
    Perform a loop until convergence is achieved
  */
  while (true) {
    /*-------------------------*
       setup PWM counter
      -------------------------*/
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_init(pwm_slice, &cfg, false);
    gpio_set_function(CAL, GPIO_FUNC_PWM);

    pwm_set_irq_enabled(pwm_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    f_hi = 0;

    /*---------------------------*
       PWM counted during 1 sec
      ---------------------------*/
    uint32_t t = time_us_32() + 2;
    while (t > time_us_32());
    pwm_set_enabled(pwm_slice, true);
    t += 1000000;
    while (t > time_us_32());
    pwm_set_enabled(pwm_slice, false);

    /*----------------------------*
       recover frequency in Hz
      ----------------------------*/
    fclk = pwm_get_counter(pwm_slice);
    fclk += f_hi << 16;
    error = fclk - Cal_freq;
    sprintf(hi,"n(%01d) cal(%lu) Hz dds(%lu) Hz err (%lu) Hz factor(%lu)\n",n, (unsigned long)Cal_freq, (unsigned long)fclk, (unsigned long)error, (unsigned long)cal_factor);
    Serial.print(hi);
    setCalibrationLED((uint16_t)error);

    if (labs(error) > int32_t(CAL_ERROR)) {
        b = !b;
        if (b) {  
          digitalWrite(TX, LOW);
        } else {
          digitalWrite(TX, HIGH);
        }
        if (error < 0) {
           cal_factor = cal_factor - CAL_STEP;
        } else {
           cal_factor = cal_factor + CAL_STEP;
        }
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    } else {
      n--;
      if (n == 0) {
        break;
      }

    }
  }  //larger while(true) loop
  
  EEPROM.put(addr, cal_factor);
  EEPROM.commit();
  setCalibrationLED(0);
  
  sprintf(hi,"Calibration procedure completed cal_factor=%d\n",cal_factor);
  Serial.print(hi);
  sprintf(hi,"Turn power-off the ADX board to start\n");
  Serial.print(hi);
  
  while (true) {
    
  }
#endif //!DDSPIO

}
//************************** [SI5351 VFO Calibration Function] ************************
void Calibration() {

#ifdef DDSPIO    //Calibration lack sense with ddsPIO enabled as it is only required when a Si5351 is used
  _INFOLIST("%s Function not available when using the ddsPIO configuration\n",__func__);
#else
  clearLED();
  
  for (int i=0;i<4;i++){

    setLED(WSPR, HIGH);
    setLED(FT8, HIGH);
    delay(100);
    
    setLED(WSPR, LOW);
    setLED(FT8, LOW);
    delay(100);

  }

  addr = 10;
  EEPROM.get(addr, cal_factor);

Calibrate:

  UP_State = digitalRead(UP);

  if (UP_State == LOW) {
    delay(50);

    UP_State = digitalRead(UP);
    if (UP_State == LOW) {
      cal_factor = cal_factor - 100;
      si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

      // Set CLK2 output
      si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration


    }
  }

  DOWN_State = digitalRead(DOWN);

  if (DOWN_State == LOW) {
    delay(50);

    DOWN_State = digitalRead(DOWN);
    if (DOWN_State == LOW) {

      cal_factor = cal_factor + 100;

      si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

      // Set CLK2 output
      si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for Calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable clock2

    }
  }

  TXSW_State = digitalRead(TXSW);

  if (TXSW_State == LOW) {
    delay(50);

    TXSW_State = digitalRead(TXSW);
    if (TXSW_State == LOW) {

      addr = 10;
      EEPROM.put(addr, cal_factor);
      EEPROM.commit();

      digitalWrite(TX, HIGH);
      delay(Bdly);
      digitalWrite(TX, LOW);
      delay(Bdly);
      digitalWrite(TX, HIGH);
      delay(Bdly);
      digitalWrite(TX, LOW);
      delay(Bdly);
      digitalWrite(TX, HIGH);
      delay(Bdly);
      digitalWrite(TX, LOW);


    }
  }

  goto Calibrate;
#endif //!DDSPIO

}

//****************************** [ End Of Calibration Function ]****************************************

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT() {

  EEPROM.begin(512);
  _INFOLIST("%s EEPROM.begin()\n", __func__);

  addr = 30;
  EEPROM.get(addr, temp);

  if (temp != 100) {

    addr = 10;
    cal_factor = 100000;
    EEPROM.put(addr, cal_factor);

    addr = 40;
    temp = 4;
    EEPROM.put(addr, temp);

    addr = 30;
    temp = 100;
    EEPROM.put(addr, temp);

    addr = 50;
    temp = 1;
    EEPROM.put(addr, temp);

    EEPROM.commit();
    _INFOLIST("%s EEPROM commit()\n", __func__);

  }

  else

  {
    //--------------- EEPROM INIT VALUES
    addr = 30;
    EEPROM.get(addr, temp);

    addr = 10;
    EEPROM.get(addr, cal_factor);

    addr = 40;
    EEPROM.get(addr, mode);

    addr = 50;
    EEPROM.get(addr, Band_slot);

  }
  Band_assign();
  //Mode_assign();

  #ifndef DDSPIO
      si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock
  #endif //!DDSPIO

  _INFOLIST("%s completed Band_slot=%d mode=%d freq=%ld\n", __func__, Band_slot, mode, freq);

}
//********************************[ END OF INITIALIZATION FUNCTION ]*************************************

