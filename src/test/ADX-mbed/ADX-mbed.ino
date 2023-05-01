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
// Board: "Raspberry Pi Pico" (Board manager is mbed rp2040 Raspberry Pi Pico)
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
// Arduino "Wire.h" I2C library(built-into arduino ide)
// ----------------------------------------------------------------------------------------------------------------------
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

//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
/*---------------------------------------------------------------------------------------------
                                 PORTING DEFINES
   The following defines are used for porting and testing purposes
  ---------------------------------------------------------------------------------------------*/
#define mbed    1

/*-----------------------------------------------
   Pico Arduino CORE includes
*/
#include <stdint.h>
#include "hardware/watchdog.h"
//#include "pico/stdlib.h"
//#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
//#include "pico/multicore.h"
//#include "hardware/adc.h"
#include "hardware/uart.h"
//#include <WiFi.h>
//#include <Time.h>
#include "ADX-mbed.h"

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "ADX_mbed"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  1.0
#define BUILD     01

/*-------------------------------------------------
   Macro expansions
*/
#define digitalWrite(x,y) gpio_put(x,y)
#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0

/*------------------------------------------------------
   Main variables
*/
char hi[128];
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t now;
time_t t_ofs;
char timestr[12];

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
                                   End of porting definitions
  =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
//*******************************[ VARIABLE DECLERATIONS ]*************************************
int mode=4;
unsigned long freq=7074000*100ULL;
unsigned long freq1=freq;
int TX_State = 0;

unsigned long F_FT8;
unsigned long F_FT4;
unsigned long F_JS8;
unsigned long F_WSPR;
int Band_slot =1;
int Band = 0;
int UP_State;
int DOWN_State;
int TXSW_State;
int Bdly = 250;
int cat_stat=0;
int cal_factor=0;

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


#define SI5351_REF 		25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz


int Band1 = 40; // Band 1 // These are default bands. Feel free to swap with yours
int Band2 = 30; // Band 2
int Band3 = 20; // Band 3
int Band4 = 10; // Band 4 //*RP2040* changed to 10m from 17m
MbedI2C i2c(I2C_SDA,I2C_SCL);
Si5351 si5351;

//*************************************[ SETUP FUNCTION ]**************************************
void setup()
{

_SERIAL.begin(115200);
_SERIAL.setTimeout(4);
while(!_SERIAL);
delay(50);
_SERIAL.flush();


  
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(TXSW,INPUT);
  /*-----
     Pull-up for input ports
     (Warning! See hardware instructions
  */

  pinMode(TXSW,INPUT_PULLUP);
  pinMode(DOWN,INPUT_PULLUP);
  pinMode(UP,INPUT_PULLUP);
  
  /*---
     Set output ports
  */
  
  pinMode(RX,OUTPUT);
  pinMode(TX,OUTPUT);
  pinMode(WSPR,OUTPUT);
  pinMode(JS8,OUTPUT);
  pinMode(FT4,OUTPUT);
  pinMode(FT8,OUTPUT);
  
  i2c.begin();
  _INFO("I/O setup completed\n");

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
  _INFO("si5351 clock initialization completed\n");

  /*--------------------
     Place the receiver in reception mode
  */
  digitalWrite(RX, LOW);

  /*--------------------
     Assign initial mode
  */
  Mode_assign();
  _INFO("setup completed successfully\n");

}

//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{

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
     This is where the actual signal processing occurs
    =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

  /*---------
     If the code fallbacks here it's because there is no audio input anymore
     then switch the transmitter off and set it to receive
  */
  if (TX_State == 1 && cat_stat == 0) {
      setTX(LOW);
  }

}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of loop()                                                      *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=



//************************************[ MODE Assign ]**********************************

void Mode_assign() {


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
  _INFO("mode=%d freq=%ld\n", mode, freq);
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

  _INFO("mode=%d freq=%ld\n",mode, freq);

}
//************************[ End of Frequency assign function ]*************************

//******************************[ Band  Assign Function ]******************************

void Band_assign() {

  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);


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

  _INFO("band_slot=%d mode=%d freq=%ld\n",Band_slot, mode, freq);
}
//***************************[ End of Band assign function ]***************************


//*******************************[ Manual TX FUNCTION ]********************************
void setTX(bool tx) {
   if (tx) {

      TX_State = 1;
      digitalWrite(RX, LOW);
      digitalWrite(TX, HIGH);

      si5351.set_freq(freq1 * 100ULL, SI5351_CLK0);
      si5351.output_enable(SI5351_CLK1, 0);   //RX off
      si5351.output_enable(SI5351_CLK0, 1);   // TX on
      _INFO("TX+\n");

   } else {

      digitalWrite(TX, LOW);
      si5351.output_enable(SI5351_CLK0, 0);   //TX off
      si5351.set_freq(freq * 100ULL, SI5351_CLK1);
      si5351.output_enable(SI5351_CLK1, 1);   //RX on
      TX_State = 0;
      digitalWrite(RX, HIGH);
      _INFO("TX-\n");
  
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
      _INFO("<UP> Band_slot=%d\n",Band_slot);

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
      _INFO("<DOWN> Band_slot=%d\n",Band_slot);

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

  Band_assign();
  _INFO("completed set Band_slot=%d\n",Band_slot);
}

//*********************************[ END OF BAND SELECT ]*****************************
//****************************** [ End Of Calibration Function ]****************************************

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT() {

  Band_assign();
  Freq_assign();
  Mode_assign();
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

  _INFO("completed Band_slot=%d mode=%d freq=%ld\n",Band_slot, mode, freq);

}
//********************************[ END OF INITIALIZATION FUNCTION ]*************************************

