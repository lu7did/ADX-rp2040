//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 1.0
//
// This is a direct port into the rp2040 architecture of the ADX_UNO firmware code (baseline version 1.1).
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
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide)
// SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*****************************************************************************************************
//* IMPORTANT NOTE: Use V2.1.3 of NT7S SI5351 Library. This is the only version compatible with ADX!!!*
//*****************************************************************************************************
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*************************************[ LICENCE and CREDITS ]*********************************************
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
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

//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
// For SI5351 VFO Calibration Procedure follow these steps:
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope
//     that can measure 1 Mhz up to 1Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton.
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL).
//     Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope.
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 8 - If you read as accurate as possible 1000000 Hz then calibration is done.
// 9 - Now we must save this calibration value to EEPROM location.
//     In order to save calibration value, press TX button briefly. TX LED will flash 3
//     times which indicates that Calibration value is saved.
// 10- Power off ADX.
//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
/*---------------------------------------------------------------------------------------------
                                 PORTING DEFINES
   The following defines are used for porting and testing purposes
  ---------------------------------------------------------------------------------------------*/
#define RP2040    1

#ifdef RP2040

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
#include <WiFi.h>
#include <Time.h>


#define PROGNAME "ADX_rp2040"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION 1.0
#define BUILD     1


#define digitalWrite(x,y) gpio_put(x,y)
#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0

#define DEBUG                1
#undef  UART
#define BAUD            115200
#define FSK_IDLE          1000    //Standard wait without signal
#define FSK_ERROR            4
#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            30  //Maximum FSK frequency computed
#define FSK_USEC       1000000    //Constant to convert T to f
#define VOX_MAXTRY          15    //VOX control cycles

extern volatile uint32_t   period;
extern bool pioirq;
extern void PIO_init();

char hi[128];
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

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
#else //!DEBUG
#define _INFOLIST(...) (void)0
#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness



#endif //RP2040 
//*******************************[ VARIABLE DECLERATIONS ]*************************************
uint32_t val;
int temp;
uint32_t val_EE;
int addr = 0;
int mode;
unsigned long freq;
unsigned long freq1;
int32_t cal_factor;
int TX_State = 0;

unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long F_FT8;
unsigned long F_FT4;
unsigned long F_JS8;
unsigned long F_WSPR;
int Band_slot;
int Band = 0;
int UP_State;
int DOWN_State;
int TXSW_State;
int Bdly = 250;
// **********************************[ DEFINE's ]***********************************************
//**********************************************************************************************
//* The following defines the ports used to connect the hard switches (UP/DOWN/TX) and the LED
//* to the rp2040 processor in replacement of the originally used for the ADX Arduino Nano or UNO
//**********************************************************************************************

#ifndef RP2040
#define UP             2  //UP Switch
#define DOWN           3  //DOWN Switch
#define TXSW           4  //TX Switch

#define TX            13  //TX LED
#define WSPR           9  //WSPR LED 
#define JS8           10  //JS8 LED
#define FT4           11  //FT4 LED
#define FT8           12  //FT8 LED

#define RX             8   //RX SWITCH
#endif //!RP2040

#ifdef RP2040
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

#define FSKpin         27      //Frequency counter algorithm, signal input PIN

/*---
    I2C
*/
#define I2C_SDA        16  //I2C SDA
#define I2C_SCL        17  //I2C SCL
#endif //RP2040
//**********************************************************************************************

#define SI5351_REF 		25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz

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

int Band1 = 40; // Band 1 // These are default bands. Feel free to swap with yours
int Band2 = 30; // Band 2
int Band3 = 20; // Band 3
int Band4 = 10; // Band 4 //*RP2040* changed to 10m from 17m

Si5351 si5351;

//*************************************[ SETUP FUNCTION ]**************************************
void setup()
{

#ifdef DEBUG
  _SERIAL.begin(115200);
  while (!_SERIAL);
  delay(200);
  _SERIAL.flush();
#endif //DEBUG


#ifndef RP2040
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(TXSW, INPUT);

  pinMode(TX, OUTPUT);
  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);
  pinMode(RX, OUTPUT);

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
#endif //!RP2040

#ifdef RP2040

  gpio_init(UP);
  gpio_init(DOWN);
  gpio_init(TXSW);

  gpio_init(RX);

  gpio_init(WSPR);
  gpio_init(JS8);
  gpio_init(FT4);
  gpio_init(FT8);
  gpio_init(TX);


  gpio_set_dir(UP, GPIO_IN);
  gpio_set_dir(DOWN, GPIO_IN);
  gpio_set_dir(TXSW, GPIO_IN);

  gpio_pull_up(TXSW);
  gpio_pull_up(DOWN);
  gpio_pull_up(UP);

  gpio_set_dir(RX, GPIO_OUT);
  gpio_set_dir(TX, GPIO_OUT);
  gpio_set_dir(WSPR, GPIO_OUT);
  gpio_set_dir(JS8, GPIO_OUT);
  gpio_set_dir(FT4, GPIO_OUT);
  gpio_set_dir(FT8, GPIO_OUT);

  gpio_init(FSKpin);
  gpio_set_dir(FSKpin, GPIO_IN);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  _INFOLIST("%s: I/O setup completed\n", __func__);

#endif //RP2040 

  /*-----------
     System initialization
  */
  INIT();

  //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  //----------------------------------------------------------------------------------
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

  _INFOLIST("%s si5351 clock initialization completed\n", __func__);


  if ( digitalRead(DOWN) == LOW ) {

    _INFOLIST("%s Calibration mode started\n", __func__);
    Calibration();
  }

  /*-------[RP2040] The COMPA interrupt isn't present on the rp2040 and thus it's replaced
           by a PIO based counting mechanism
  */
#ifndef RP2040
  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1 << ACIC); // Analog Comparator Capture Input

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
#endif //!RP2040 avoid using COMPA interrupt based counting method as it is not available on rp2040
  /*--------*/

#ifdef RP2040
  PIO_init();
  _INFOLIST("%s PIO sub-system initialized\n", __func__);
#endif //Initialize the PIO based counting method used on rp2040

  digitalWrite(RX, LOW);

  Mode_assign();
  _INFOLIST("%s setup completed successfully\n", __func__);

}

//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{

  UP_State = digitalRead(UP);
  DOWN_State = digitalRead(DOWN);

#ifdef DEBUG
  if (UP_State == LOW || DOWN_State == LOW) {
    _INFOLIST("%s UP_State=%d DOWN_State=%d\n", __func__, UP_State, DOWN_State);
  }
#endif //DEBUG


  if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
    delay(100);
    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);
    if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
      Band_Select();
    }
  }



  if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0)) {
    delay(100);
    UP_State = digitalRead(UP);
    if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0)) {
      mode = mode - 1;
      if (mode < 1) {
        mode = 4;
      }



      addr = 40;
      EEPROM.put(addr, mode);

#ifdef RP2040
      EEPROM.commit();   //rp2040 doesn't have any EEPROM, the core library emulates EEPROM on flash memory, but it requires a commit() to set
#endif //RP2040          
      Mode_assign();
    }
  }

  DOWN_State = digitalRead(DOWN);

  if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0)) {
    delay(50);

    DOWN_State = digitalRead(DOWN);
    if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0)) {
      mode = mode + 1;

      if (mode > 4) {
        mode = 1;
      }

      addr = 40;
      EEPROM.put(addr, mode);

#ifdef RP2040
      EEPROM.commit(); //rp2040 doesn't have an EEPROM, the core library emulates EEPROM on flash memory and requires a commit() to set values permanently
#endif //RP2040          

      Mode_assign();

    }
  }

  TXSW_State = digitalRead(TXSW);

  if ((TXSW_State == LOW) && (TX_State == 0)) {
    delay(50);

    TXSW_State = digitalRead(TXSW);
    if ((TXSW_State == LOW) && (TX_State == 0)) {
      Mode_assign();
      ManualTX();
    }
  }

  /*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
     This is the audio frequency counting algorithm the ADX board uses to obtain the audio tone to be sent
     during the ft8 frame. Unfortunately the interrupt used for this algorithm isn't available on the rp2040
     therefore a suitable replacement using a PIO microcode has been built instead.
     The entire code segment is nullified when compiling with the RP2040 configuration
    =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#ifndef RP2040

  unsigned int d1, d2;
  int FSK = 10;
  int FSKtx = 0;
  while (FSK > 0) {
    TCNT1 = 0;
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }  while ((ACSR & (1 << ACO)) == 0) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    TCNT1 = 0;
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    d1 = ICR1;
    while ((ACSR & (1 << ACO)) == 0) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    d2 = ICR1;
    if (TCNT1 < 65000) {
      unsigned long codefreq = 1600000000 / (d2 - d1);
      if (codefreq < 350000) {
        if (FSKtx == 0) {
          TX_State = 1;
          digitalWrite(TX, HIGH);
          digitalWrite(RX, LOW);

          si5351.output_enable(SI5351_CLK1, 0);   //RX off
          si5351.output_enable(SI5351_CLK0, 1);   // TX on
        }
        si5351.set_freq((freq * 100 + codefreq), SI5351_CLK0);

        FSKtx = 1;
      }
    }
    else {
      FSK--;
    }
  }
#endif //!RP2040
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
#ifdef RP2040
  int FSK = VOX_MAXTRY;
  int FSKtx = 0;

  int k = 0;

  while ( FSK > 0 ) {                                //Iterate up to 10 times looking for signal to transmit

    if (pioirq == true) {

      pioirq = false;
      FSK = VOX_MAXTRY;
      if (period > 0) {
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

          TX_State = 1;
          digitalWrite(TX, HIGH);
          digitalWrite(RX, LOW);

          si5351.output_enable(SI5351_CLK1, 0);   //RX off
          si5351.output_enable(SI5351_CLK0, 1);   // TX on

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
          si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0);
          prevfreq = codefreq;
        }
      }
    } else {
      /*--------------------
        Waiting for signal, rp2040 is way faster than ATMEGA328p thus a delay is needed
        --------------------*/
      uint32_t tcnt = time_us_32() + uint32_t(FSK_IDLE);
      while (time_us_32() < tcnt);
      FSK--;
      if (FSK == 0 && TX_State == 1) {
        _INFOLIST("%s VOX turned OFF\n", __func__);
      }
    }
  }
#endif //End the processing made by the FREQPIO algorithm    
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

  /*---------
     If the code fallbacks here it's because there is no audio input anymore
     then switch the transmitter off and set it to receive
  */

  digitalWrite(TX, 0);

  si5351.output_enable(SI5351_CLK0, 0);   //TX off
  si5351.set_freq(freq * 100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX on
  TX_State = 0;
  digitalWrite(RX, HIGH);

}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of loop()                                                      *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                       Support functions                                                    *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


//************************************[ MODE Assign ]**********************************

void Mode_assign() {

  addr = 40;
  EEPROM.get(addr, mode);


  if ( mode == 1) {
    freq1 = F_WSPR;
    digitalWrite(WSPR, HIGH);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);

  }

  if ( mode == 2) {
    freq1 = F_JS8;
    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, HIGH);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);



  }

  if ( mode == 3) {
    freq1 = F_FT4;

    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, HIGH);
    digitalWrite(FT8, LOW);


  }
  if ( mode == 4) {
    freq1 = F_FT8;

    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, HIGH);


  }
  freq = freq1;
  //freq = freq1 - 1000;

  _INFOLIST("%s mode=%d freq=%ld\n", __func__, mode, freq);
}

//********************************[ END OF MODE ASSIGN ]*******************************

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign() {


  //---------- 80m/3.5Mhz
  if (Band == 80) {

    F_FT8 = 3573000;
    F_FT4 = 3575000;
    F_JS8 = 3578000;
    F_WSPR = 3568600;
  }

  //---------- 40m/7 Mhz
  if (Band == 40) {

    F_FT8 = 7074000;
    F_FT4 = 7047500;
    F_JS8 = 7078000;
    F_WSPR = 7038600;
  }


  //---------- 30m/10 Mhz
  if (Band == 30) {

    F_FT8 = 10136000;
    F_FT4 = 10140000;
    F_JS8 = 10130000;
    F_WSPR = 10138700;
  }


  //---------- 20m/14 Mhz
  if (Band == 20) {

    F_FT8 = 14074000;
    F_FT4 = 14080000;
    F_JS8 = 14078000;
    F_WSPR = 14095600;
  }


  //---------- 17m/18 Mhz
  if (Band == 17) {

    F_FT8 = 18100000;
    F_FT4 = 18104000;
    F_JS8 = 18104000;
    F_WSPR = 18104600;
  }

  //---------- 15m/ 21Mhz
  if (Band == 15) {

    F_FT8 = 21074000;
    F_FT4 = 21140000;
    F_JS8 = 21078000;
    F_WSPR = 21094600;
  }

  //---------- 10m/ 28Mhz
  if (Band == 10) {

    F_FT8 = 28074000;
    F_FT4 = 28180000;
    F_JS8 = 28078000;
    F_WSPR = 28124600;
  }
  _INFOLIST("%s mode=%d freq=%ld\n", __func__, mode, freq);

}
//************************[ End of Frequency assign function ]*************************

//******************************[ Band  Assign Function ]******************************

void Band_assign() {
  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);


  addr = 50;
  EEPROM.get(addr, Band_slot);

  if (Band_slot == 1) {
    Band = Band1;

    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
  }

  if (Band_slot == 2) {
    Band = Band2;

    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
  }

  if (Band_slot == 3) {
    Band = Band3;

    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
  }

  if (Band_slot == 4) {
    Band = Band4;

    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
  }


  delay(1000);
  Freq_assign();
  Mode_assign();

  _INFOLIST("%s band_slot=%d mode=%d freq=%ld\n", __func__, Band_slot, mode, freq);
}
//***************************[ End of Band assign function ]***************************


//*******************************[ Manual TX FUNCTION ]********************************
void ManualTX() {

  digitalWrite(RX, LOW);
  si5351.output_enable(SI5351_CLK1, 0);   //RX off

  _INFOLIST("%s TX+\n", __func__);


TXON:

  TXSW_State = digitalRead(TXSW);


  digitalWrite(TX, 1);
  si5351.set_freq(freq1 * 100ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   //TX on
  TX_State = 1;

  if (TXSW_State == HIGH) {
    goto EXIT_TX;

  }
  goto TXON;

EXIT_TX:
  digitalWrite(TX, 0);
  si5351.output_enable(SI5351_CLK0, 0);   //TX off
  TX_State = 0;
  _INFOLIST("%s TX-\n", __func__);

}

//********************************[ END OF Manual TX ]*********************************

//******************************[ BAND SELECT Function]********************************
void Band_Select() {

  digitalWrite(TX, 1);
  addr = 50;
  EEPROM.get(addr, Band_slot);

  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);


  if (Band_slot == 1) {


    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
    digitalWrite(WSPR, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    delay(Bdly);
  }

  if (Band_slot == 2) {

    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
    digitalWrite(JS8, HIGH);
    delay(Bdly);
    digitalWrite(JS8, LOW);
    delay(Bdly);
  }

  if (Band_slot == 3) {

    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
    digitalWrite(FT4, HIGH);
    delay(Bdly);
    digitalWrite(FT4, LOW);
    delay(Bdly);
  }

  if (Band_slot == 4) {

    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(FT8, LOW);
    delay(Bdly);
  }

Band_cont:

  if (Band_slot == 1) {


    digitalWrite(WSPR, HIGH);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);
  }

  if (Band_slot == 2) {


    digitalWrite(JS8, HIGH);
    digitalWrite(WSPR, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);

  }

  if (Band_slot == 3) {
    digitalWrite(FT4, HIGH);
    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, LOW);
    digitalWrite(FT8, LOW);



  }

  if (Band_slot == 4) {
    digitalWrite(JS8, LOW);
    digitalWrite(WSPR, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, HIGH);

  }


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

#ifdef RP2040
  EEPROM.commit();
#endif //RP2040

  Band_assign();
  _INFOLIST("%s completed set Band_slot=%d\n", __func__, Band_slot);


}

//*********************************[ END OF BAND SELECT ]*****************************



//************************** [SI5351 VFO Calibration Function] ************************

void Calibration() {

  digitalWrite(FT8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(WSPR, LOW);

  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);
  delay(100);

  digitalWrite(WSPR, LOW);
  digitalWrite(FT8, LOW);
  delay(100);


  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);
  delay(100);

  digitalWrite(WSPR, LOW);
  digitalWrite(FT8, LOW);
  delay(100);

  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);
  delay(100);

  digitalWrite(WSPR, LOW);
  digitalWrite(FT8, LOW);
  delay(100);

  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);
  delay(100);

  digitalWrite(WSPR, LOW);
  digitalWrite(FT8, LOW);
  delay(100);

  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);

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


#ifdef RP2040
      EEPROM.commit();
#endif //DEBUG


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
}

//****************************** [ End Of Calibration Function ]****************************************

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT() {


#ifdef RP2040
  EEPROM.begin(512);
  _INFOLIST("%s EEPROM.begin()\n", __func__);
#endif //RP2040 



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


#ifdef RP2040
    EEPROM.commit();
    _INFOLIST("%s EEPROM commit()\n", __func__);
#endif //RP2040


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
  Freq_assign();
  Mode_assign();
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

  _INFOLIST("%s completed Band_slot=%d mode=%d freq=%ld\n", __func__, Band_slot, mode, freq);

}
//********************************[ END OF INITIALIZATION FUNCTION ]*************************************
