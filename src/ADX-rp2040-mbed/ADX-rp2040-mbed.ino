//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                            Pedro (Pedro Colla) - LU7DZ - 2022,2023
//
//                                         Version 4.0
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//* Based on ADX-rp2040 by Pedro Colla LU7DZ (2022)
//* Originally ported from ADX_UnO_V1.3 by Barb Asuroglu (WB2CBA)
//
// This is experimental code trying to port the ADX-rp2040 code to the Arduino IDE mbed core in order 
// to implement the link with WSJT-X thru an USB audio/serial port
//
// This code relies heavily on the great work from Hitoshi, JE1RAV at the QP-7C_RP2040 transceiver and
// his generous sharing of insights and code leading to this solution.
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*
//* Code excerpts from different sources
//*
//* excerpts taken from Orange_Thunder by Pedro Colla (LU7DID/LU7DZ) 2018
//* excerpts taken from QP-7C_RP2040 by Hitoshi (JE1RAV)
//* code refactoring made by Pedro Colla (LU7DZ) 2022
//*
//*********************************************************************************************************
// Required Libraries and build chain components
//
// Created with Arduino IDE using the Arduino Mbed OS RP2040 core 
// 
//                                     *******************************************
//                                     *                Warning                  *
//                                     *******************************************
//
// This firmware is meant to be used with an ADX board where the Arduino Nano or Arduino Uno processor has been replaced
// by a raspberry pi pico board plus addicional voltage and signal conditioning circuits, please see the host site
// https://github.com/lu7did/ADX-rp2040 for construction details and further comments.
//
// This firmware is meant to be compiled using the latest Arduino IDE environment with the following parameters
//
// Arduino Mbed OS RP2040 boards
// Board: "Raspberry Pi Pico"
//
// The firmware has been developed with a std Raspberry Pi Pico version using the Arduino IDE mbed core
//*****************************************************************************************************
// Arduino "Wire.h" I2C library         (built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library    (built-into arduino ide)
// To be installed using the Arduino IDE Library Manager
// Etherkit Si5351 is used thru a modified version contained with this package
//                The reason for the local copy is to set the SDA/SCL ports as 16/17 which are required
//                by the board and can not be set externally to the library under the mbed core
//                SI5351       (https://github.com/etherkit/Si5351Arduino) Library by Jason Mildrum (NT7S) 
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
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
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "ADX_rp2040-mbed"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  "4.0"
#define BUILD     "00"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#include "hardware/adc.h"
#include "PluggableUSBAudio.h"
#include "si5351.h"
#include "Wire.h"
#include "ADX-rp2040.h"

//*--------------------------------------------------------------------------------------------
//* Definition of NeoPixel RGB management library
//*--------------------------------------------------------------------------------------------
#ifdef PixiePico
#include <NeoPixelConnect.h>
NeoPixelConnect led(16, 1, pio0, 0);  // creamos la instacia para utilizar el led, con el nombre de led
#endif //Support for NeoPixel RGB led for signaling

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//.                Master clock
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
Si5351 si5351;
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                Transceiver Frequency management memory areas
//*
//* Notes on PixiePico project
//*
//* PixiePico is a single band transceiver, originally aimed for the 28 MHz band
//* however, the band switch capability of the firmware is left in a way that it can be
//* programmed by firmware to operate in other band (assuming the LPF of the Pixie transceiver)
//* is adjusted accordingly by changing the value of the Band_slot variable to point to the
//* correct entry in the Bands table.
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
const uint16_t Bands[BANDS] = {40,30,20,10};
const unsigned long slot[MAXBAND][3] = { {3573000UL,3500000UL,3800000UL},         //80m [0]
                                         {5357000UL,5351000UL,5356000UL},         //60m [1]
                                         {7074000UL,7000000UL,7300000UL},         //40m [2]
                                         {10136000UL,10100000UL,10150000UL},      //30m [3]
                                         {14074000UL,14000000UL,14350000UL},      //20m [4]
                                         {18100000UL,18068000UL,18168000UL},      //17m [5]
                                         {21074000UL,21000000UL,21450000UL},      //15m [6]
                                         {24915000UL,24890000UL,24990000UL},      //12m [7]
                                         {28074000UL,28000000UL,29700000UL}};     //10m [8]
#ifdef ADX-rp2040                                         

int Band_slot = 3;           // This is he default starting band 1=40m, 2=30m, 3=20m, 4=10m
uint64_t RF_freq=7074000;    // RF frequency (Hz)

#else

int Band_slot = 4;           //Default for PixiePico is 28MHz
uint64_t RF_freq=28074000;   // RF frequency (Hz)
uint8_t rxMask =LED_DIMM;

#endif //ADX-rp2040

uint16_t band;
bool SI473x_enabled=false;

#ifdef SUPERHETERODYNE
int64_t BFO_freq = FREQ_BFO;  
#else
int64_t BFO_freq = 0;
#endif //SUPERHETERODYNE

int C_freq = 0;  //FREQ_x: In this case, FREQ_0 is selected as the initial frequency.
int Tx_Status = 0; //0=RX, 1=TX
int Tx_Start = 0;  //0=RX, 1=TX
int not_TX_first = 0;
uint32_t Tx_last_mod_time;
uint32_t Tx_last_time;

time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
time_t now;
char timestr[12];

/*------------------------------------------------------
 *   Internal clock handling
 */
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time

uint32_t trssi = time_us_32();

/*-----------------------------
  Audio signal frequency determination
 */
 
int16_t mono_prev=0;  
int16_t mono_preprev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
int32_t cycle_frequency[34];

/*-----------------------------
  ADC processing
*/
int16_t adc_offset = 0;          //Receiver offset

bool     adc_high;
bool     adc_low;
   
/*-------------------------------*
   Computed frequency limits
*/
double   ffmin;
double   ffmax;    
uint16_t adc_min;
uint16_t adc_max;
uint16_t adc_zero;
uint16_t adc_uh;
uint16_t adc_ul;

/*-----------------------------
  USB Audio definition and control blocks
 */

USBAudio audio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);

int16_t monodata[SAMPLESIZE];
uint16_t pcCounter=0;
uint16_t nBytes=0;
static uint8_t readBuffer[192];  //48 sampling (=  0.5 ms at 48000 Hz sampling) data sent from PC are recived (16bit stero; 48*2*2).
int16_t writeBuffer16[SAMPLESIZE*2];       //48 sampling date are written to PC in one packet (stero).
uint16_t writeCounter=0;
int prevRSSI=0;
bool USBAudio_read;
int64_t last_audio_freq=0;

/*-----------------------------
  Memory generic areas
 */
static uint8_t buf[128];
char hi[256];
int i=0;
bool flagFirst=true;

int cat_stat = 0;
int CAT_mode = 2;   

#ifdef CAT

/*-------------------------------------------------------------------------------------------
  CAT Processing loop
*/
#define CATCMD_SIZE          64

char CATbuf[CATCMD_SIZE];
char CATResp[CATCMD_SIZE];
char CATCmd[CATCMD_SIZE];
char serialBuf[CATCMD_SIZE*2];
char resp[CATCMD_SIZE*2];
bool ignoreFA=false;

int  CATT1=0;
int  CATT2=0;
bool flipLED=false;

#endif //CAT


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                Band and frequency management
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*--------------
  given the current slot return the band associated with it from bands[]
*/  
uint16_t slot2band(uint16_t s) {

if (s >= 0 && s <= (BANDS-1)) {
   return Bands[s];  
}

#ifdef ADX-rp2040
return 40;
#else
return 10;
#endif //ADX-rp2040
  
}
/*---------------
  given the band return the frequency associated with it from freqs[]
*/  
uint64_t band2freq(uint16_t b) {

uint16_t bx=b;

#ifdef ADX-rp2040
if (b >160 || b < 10) {
   bx=40;
}
#else
   bx=10;
#endif //ADX-rp2040

uint16_t j=Band2Idx(bx);
return slot[j][0];
}
/*------------------------
  given the band return the slot
*/  
uint16_t band2slot(uint16_t b) {

#ifdef ADX-rp2040
if (b >160 || b < 10) {
   return 0;
}
#else
return 3;
#endif //ADX-rp2040


for (int i=0;i<BANDS;i++) {
   if (Bands[i]==b) {
       return i;
   }  
}
return i;

}
/*-----------------------------------
  Given the (aprox) frequency compute the
  nearest ham radio band
*/
uint16_t freq2band(uint64_t f) {

if (f>= 1800000 && f<= 1900000) {return 160;}
if (f>= 3500000 && f<= 3800000) {return  80;}
if (f>= 7000000 && f<= 7300000) {return  40;}
if (f>=10000000 && f<=10300000) {return  30;}
if (f>=14000000 && f<=14300000) {return  20;}
if (f>=18000000 && f<=18300000) {return  17;}
if (f>=21000000 && f<=21300000) {return  15;}
if (f>=24000000 && f<=24300000) {return  12;}
if (f>=28000000 && f<=29500000) {return  10;}

#ifdef ADX-rp2040
return 40;
#else
return 10;
#endif //ADX-rp2040

}
/*----------------------------------------
 * min Freq
 *----------------------------------------*/
uint16_t minFreq(uint16_t i){
  unsigned long s=slot[i][1];
  unsigned long f=s/1000;
  _INFO("index received(%d) s(%ld) freq(%ld)\n",i,s,f);
  uint16_t mf=uint16_t(f);
  _INFO("Slot[%d] f=%d KHz\n",i,mf);
  delay(1000);
  return mf;
}
/*----------------------------------------
 * max Freq
 *----------------------------------------*/
uint16_t maxFreq(uint16_t i){
  unsigned long s=slot[i][2];
  unsigned long f=s/1000;
  _INFO("index received(%d) s(%ld) freq(%ld)\n",i,s,f);
  uint16_t mf=uint16_t(f);
  _INFO("Slot[%d] f=%d KHz\n",i,mf);
  delay(1000);

  return mf;
}
/*----------------------------------------
 * current Freq
 *----------------------------------------*/
uint16_t currFreq(uint16_t i){
  unsigned long s=slot[i][0];
  unsigned long f=s/1000;
  _INFO("index received(%d) s(%ld) freq(%ld)\n",i,s,f);
  uint16_t mf=uint16_t(f);
  _INFO("Slot[%d] f=%d KHz\n",i,mf);
  delay(1000);
  return mf;
}

//*********************************[ INITIALIZATION FUNCTION ]******************************************
uint16_t Band2Idx(uint16_t b) {
  uint16_t i=2;
  switch (b) {
    case 80 : i=0;break;
    case 60 : i=1;break;
    case 40 : i=2;break;    //This is the default band in case the argument is wrong
    case 30 : i=3;break;
    case 20 : i=4;break;
    case 17 : i=5;break;
    case 15 : i=6;break;
    case 12 : i=7;break;
    case 10 : i=8;break;
  }
  return i;

}
#ifdef PixiePico
/*---------
  showRGB
  set the RGB LED
*/
void showRGB(uint8_t r,uint8_t g,uint8_t b,uint8_t RGBmode) {
  led.neoPixelClear(true); // limpiamos el led
  delay(10);
  led.neoPixelFill(r & RGBmode, g & RGBmode, b & RGBmode, true); // le pasamos el valor de RGB del color que queremos
}
void showTX() {
  _INFO(" TX RGB=BLUE mode cat_stat=%d\n",cat_stat);
  showRGB(255,0,0,rxMask);
}
void showRX() {
  #ifdef CAT
    showRGB(255,255,0,rxMask);
  _INFO(" RX RGB=YELLOW mode cat_stat=%d\n",cat_stat);

    //if (cat_stat == 0) {
    //  showRGB(255,255,0,rxMask);
    //} else {
    //  showRGB(192,192,192,rxMask);
    //}
  #else  
    showRGB(0,0,255,rxMask);
  _INFO(" RX RGB=BLUE mode cat_stat=%d\n",cat_stat);

  #endif //CAT
}
void showSetup() {
  _INFO("rxMask=%d\n",rxMask);
  for (int i=0;i<LED_BLINK;i++) {
     showRGB(255,255,0,rxMask);
     delay(100);
     showRGB(0,255,255,rxMask);
     delay(100);
  }
}
#endif //PixiePico
/*---------
  get a Switch value with de-bouncing
*/  
bool getSW(uint8_t k) {

  if (digitalRead(k)==LOW) {
    uint32_t t = time_us_32() + ONE_MSEC;
    while (t > time_us_32());
    return digitalRead(k);
  }
  return HIGH;
}
/*----------------
  set slot changing band and frequency accordingly
*/  
void setSlot(uint16_t s) {

  band=slot2band(s);
  RF_freq=band2freq(band);

  _INFO("Slot=(%d) band(%d) Freq(%llu)\n",s,band,RF_freq);

/*------------
  If present and already enabled change the Si473x frequency accordingly
*/
#ifdef RX_SI473X
  if (SI473x_enabled) {
     SI473x_setFrequency(s);
  }
#endif //RX_SI473X  

  setLEDbyband(band);
  _INFO("Set slot[%d] band[%d] mts freq=%ul Hz\n",Band_slot,band,RF_freq);

}
/*-------------
  Set a given LED set by the band
*/
void setLEDbyband(uint16_t b) {
  uint16_t s=band2slot(b);
  setLEDbyslot(s);
}  
/*----------
  Handle switch commands on the board
*/
void handleSW() {

   if (cat_stat == 1) {
      //_INFO("CAT operating\n");
      return;
   }

#ifdef ADX-rp2040

   if (getSW(UP)==LOW) {
      _INFO("(UP) Button\n");
      while(getSW(UP)==LOW);
      Band_slot=(Band_slot+1);
      if (Band_slot>BANDS-1) { 
        Band_slot=0;
      }
      setSlot(Band_slot);     
      si5351_setFreq(RF_freq);
   }

   if (getSW(DOWN)==LOW) {
      _INFO("(DOWN) button\n");
      while(getSW(DOWN)==LOW);
      int new_slot=Band_slot-1;
      if (new_slot<0) {
         Band_slot = BANDS-1;
      }  else {
         Band_slot=new_slot;
      }
      setSlot(Band_slot);
      si5351_setFreq(RF_freq);
   }

   #endif //ADX-rp2040

/*--------------------------------------------------------------------
  when the TXSW switch is presed while in operation the transceiver is
  placed in transmission mode (for testing purposes)
*/  

  if (getSW(TXSW)==LOW) {
     Tx_Status=0;
     transmit(int64_t(AF_TONE));
     while(getSW(TXSW)==LOW);
     receive();     

  }

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                USB UAC2 managing blocks
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

/*-------------
  UAC2 initialization
 */
void USB_UAC() {
 for (int i = 0; i < SAMPLESIZE; i++) {
   monodata[i] = 0;
 }
}
/*
USBAudioRead
Read the USB buffer and takes 48 samples, average left and right channel into a mono sample

** Fix suggested by Hitoshi to avoid the stack issue found on earlier versions, increase the buffer and 
   avoid addressing the buffer using a int16_t* variable

*/
void USBAudioRead() {
  USBAudio_read = audio.read(readBuffer, sizeof(readBuffer));
  int32_t monosum=0;
  if (USBAudio_read) { 
    for (int i = 0; i < SAMPLESIZE ; i++) {
      int16_t outL = ((int16_t)readBuffer[4*i]) + ((int16_t)readBuffer[4*i+1] << 8);
      int16_t outR = ((int16_t)readBuffer[4*i+2]) + ((int16_t)readBuffer[4*i+3] << 8);
      int16_t mono = (outL+outR)/2;
      monosum += mono;
      monodata[i] = mono;
    }
  }
  if (monosum == 0) {
     USBAudio_read=0;
  }   
}
/*------------------------------------------------------
  Write the buffer from the ADC into the USB
*/
void USBAudioWrite(int16_t left,int16_t right) {
  if(nBytes>191){
    uint8_t *writeBuffer =  (uint8_t *)writeBuffer16;
    audio.write(writeBuffer, 192);
    writeCounter =0;
    nBytes =0;
  }
  writeBuffer16[writeCounter]=left;
  writeCounter++;
  nBytes+=2;
  writeBuffer16[writeCounter]=right;
  writeCounter++;
  nBytes+=2;
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                Si5351 sub-system
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*---------
  set Si5351 frequency
*/
void si5351_setFreq(uint64_t f) {

  _INFO("Freq=%llu BFO=%llu\n",f,BFO_freq);
  si5351.set_freq(f*100ULL, SI5351_CLK0);              //for TX
  si5351.set_freq((f-BFO_freq)*100ULL, SI5351_CLK1);   //for RX
  si5351.set_freq(BFO_freq*100ULL, SI5351_CLK2);       //for BFO

}
/*---------
  init Si5351 clock
*/
void si5351_init() {

  bool si5351_rc=si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 

  if (!si5351_rc) {
     _INFO("** ERROR ** no Si5351 clock found\n");
  } else {
     _INFO("Si5351 clock found and initialized Ok\n");
  }
  int32_t cal_factor = -11800;

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);

/*------------
  Set TX clock
*/
  //si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);  //for TX
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0);   //Disabled on startup
/*------------
  Set RX clock (BFO_freq should be zero if not superheterodyne)
*/  
  //si5351.set_freq((RF_freq-BFO_freq)*100ULL, SI5351_CLK1);  //for RX
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK1, 1);  //Enabled on startup

/*------------
  Set BFO (if not superheterodyne set to zero)
*/  
  //si5351.set_freq(BFO_freq*100ULL, SI5351_CLK2);  //for BFO
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK2, 1);   //Enabled on startup
  si5351_setFreq(RF_freq);
  _INFO("Master clock sub-system initializationn completed rc(%s)!\n",BOOL2CHAR(si5351_rc));
}  
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                Board management
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void init_IO() {

  pinMode(A0, INPUT);            //ADC input pin
  pinMode(TXSW, INPUT_PULLUP);
  pinMode(RXSW, OUTPUT);

#ifdef ADX-rp2040

  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);

  pinMode(TX, OUTPUT);

  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);

#endif //ADX-rp2040


/*--------------------
     Initialize starting mode (RX on,TX off, all LED off)
  */


#ifdef ADX-rp2040
  digitalWrite(RXSW, HIGH);
  digitalWrite(TX, LOW);
  digitalWrite(WSPR,LOW);
  digitalWrite(FT8,LOW);
  digitalWrite(FT4,LOW);
  digitalWrite(JS8,LOW);

#else
  digitalWrite(RXSW,LOW);
#endif //ADX-rp2040

  _INFO("Board I/O initialized\n");

}
/*-------------
  Set a given LED set by the slot
*/  
void setLEDbyslot(uint16_t s) {

#ifdef ADX-rp2040
  for (int i=0;i<4;i++) {
      if (i==s) {
         digitalWrite(7-i,HIGH);
      } else {
         digitalWrite(7-i,LOW);
      }
  }
#else

  showRGB(0,0,255,rxMask);

#endif //ADX-rp2040

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                               ADC setup & processing
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*-------------------------------
  initialize ADC port
*/
void adc_setup() {

  adc_init();
  adc_select_input(0);                        //ADC input pin A0
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo
  adc_run(true);                              //ADC free running
  _INFO("ADC setup completed\n");


}
/*--------------------------------
  Collect samples from the receiving port
*/
int16_t adc() {
  int16_t adc = 0;
  for (int i=0;i<24;i++){                    // 192kHz/24 = 8kHz
  
/*--------------------------------
  This needs to be reviewed for the ADX-rp2040 board
  considering whether the receiver is a CD2003GP or a Si473x
*/    
  #ifdef SUPERHETERODYNE
    adc += adc_fifo_get_blocking() -1862 ;   // read from ADC fifo (offset about 1.5 V: DET OUT)
  #else
    adc += adc_fifo_get_blocking() -745 ;    // read from ADC fifo (offset about 0.6 V: AM MIX OUT)
  #endif
  
  }  
  return adc;
}
/*------------------------------------------------------------------------------------------*
   calibrateADC
   Calibrate the ADC zero level
*/
uint16_t adc_calibrate(uint16_t min, uint16_t max) {
  return uint16_t((adc_max - adc_min) * 1.0 / 2.0) + adc_min;
}
/*-------------------------------------------------------------------------------------------*   
   adc_reset
   restore all calibration values
*/
void adc_reset() {
  adc_min  = ADCMAX;
  //qadc_max  = ADCMIN;
  adc_zero = ADCZERO;
  adc_uh   = adc_zero * 110 / 100;
  adc_ul   = adc_zero * 90 / 100;
//  ffmin    = FSKMAX;
//  ffmax    = FSKMIN;
  _INFO("Timeout, recalibrate input level");
}
/*------------------------------------------------------------------------------------------*
   adc_sample
   collect an ADC sample running free.
   update the minimum and maximum
*/
uint16_t adc_sample() {
  uint16_t v = adc_read();
  if (v > adc_max) {
    adc_max = v;
    adc_zero = adc_calibrate(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;  
    _INFO("calibration (max) adc_max=%d adc_min=%d adc_Zero=%d\n",adc_max, adc_min, adc_zero);
     return v;
  }
  if (v <= adc_min) {
    adc_min = v; 
    adc_zero = adc_calibrate(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;
   
    _INFO("calibration (min) adc_max=%d adc_min=%d adc_Zero=%d\n",adc_max, adc_min, adc_zero);
    return v;
  }
  if (v >= adc_uh) {
    adc_high = true;
  }
  if (v <= adc_ul) {
    adc_low = true;
  }
  return  v;  
} 

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                               setup cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup() {
  
/*----------
    Serial port initialization
   */
  Serial.begin(115200);

  #ifndef CAT
  while(!Serial);
  #endif //CAT
  
  Serial.setTimeout(100);
  Serial.flush();

#ifdef ADX-rp2040
  _INFO("ADX-rp2040-mbed digital transceiver v(%s) build(%s)\n",VERSION,BUILD);
#else
  _INFO("PixiePico digital transceiver v(%s) build(%s)\n",VERSION,BUILD);
#endif //ADX-rp2040

/*-----------
  Init I/O
*/
  init_IO();
  showSetup();

/*-----------
  Set initial band, slot, frequency and set LED
*/
  setSlot(Band_slot);

//ADC initialize ----- 
  adc_setup();

/*-----------
  Initialize Si5351
 */
  si5351_init();

#ifdef RX_SI473X
/*-----------
  Initialize Si473x chipset
*/
  _INFO("Si473x chip initialization\n");
  delay(500);
  SI473x_Setup();
#else
#ifdef ADX-rp2040
  _INFO("CD2003GP chip receiver selected\n");
#else
  _INFO("Pixie receiver selected\n");
#endif   
#endif //RX_SI473X  
/*------------
  USB Audio initialization
*/
  USB_UAC();
  _INFO("Transceiver USB Sub-system ready\n");


/*-------------
  read the DC offset value of ADC input (about 0.6V)----- 
*/  
  delay(500);
  adc_fifo_drain ();
  adc_offset = adc();

/*-------------
  Place the transceiver in receiver mode
*/
receive();
_INFO("Transceiver in receiver mode, ready\n");    
  
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                               loop cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void loop() {

/*--------
  Handle board switches
*/
  handleSW();

/*--------
  Handle CAT commands
*/
  cat();
/*--------
  Tx_Status==0 RX mode -- Tx_Status!=0 TX mode
 */

  if (Tx_Start==0) {
     receiving();
  } else {
     transmitting();
  }

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                            end of loop cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

/*-----------------------------------------------------------------------------------------------------------------------------------*
  RX/TX Cycle and VOX control logic
 *-----------------------------------------------------------------------------------------------------------------------------------*/
/*------------
  Main transmitting control cycle
 */


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
 Transmitting
 This function checks if there is USB data to transmit, applying some heuristics to understand when the data stream ceases
 and thus a switch to the receiving mode is needed.
 When data stream is available a zero crossing algorithm (see QDX algorithm by Hans Summers) is used to get the period
 and then the frequency of the signal, a couple of heuristics (min & max, minimum change, minimum latency) are applied
 to reject noise or transient frequency changes.
*/
void transmitting(){
  int64_t audio_freq;
  if (USBAudio_read) {
    for (int i=0;i<SAMPLESIZE ;i++){
      int16_t mono = monodata[i];
      if ((mono_prev < 0) && (mono >= 0)) {
        if ((mono == 0) && (((float)mono_prev * 1.8 - (float)mono_preprev < 0.0) || ((float)mono_prev * 2.02 - (float)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
          Tx_Start = 0;
          last_audio_freq=0;
          break;
        }
        int16_t difference = mono - mono_prev;

        /*------ (Original from Hitoshi-san code)
        // x=0付近のsin関数をテーラー展開の第1項まで(y=xで近似）
        // Sin(x) function near x=0 can be approximated with the the first term of Taylor series (y=x approximation)
        */

        float delta = (float)mono_prev / (float)difference;
        float period = (1.0 + delta_prev) + (float)sampling - delta;
        audio_freq = AUDIOSAMPLING*100.0/period; // in 0.01Hz    

        if ((audio_freq>FSK_MIN) && (audio_freq<FSK_MAX)){
          cycle_frequency[cycle]=audio_freq;
          cycle++;
        }
        delta_prev = delta;
        sampling=0;
        mono_preprev = mono_prev;
        mono_prev = mono;     
      }
      else if ((not_TX_first == 1) && (mono_prev == 0) && (mono == 0)) {        //Detect non-transmission
        Tx_Start = 0;
        last_audio_freq=0;
        break;
      }
      else {
        sampling++;
        mono_preprev = mono_prev;
        mono_prev = mono;
      }
    }
    if (Tx_Start == 0){
      cycle = 0;
      receive();
      return;
    }
    if ((cycle > 0) && (millis() - Tx_last_mod_time > 5)){          //inhibit the frequency change faster than 5mS
      audio_freq = 0;
      for (int i=0;i<cycle;i++){
        audio_freq += cycle_frequency[i];
      }

      audio_freq = audio_freq / cycle;
      long unsigned freqdiff=abs((long int)audio_freq-(long int)last_audio_freq);

      if (freqdiff > FSK_MIN_CHANGE) {
         //_INFO("FSK=%ld Hz (diff=%lu)\n",(long int)audio_freq,freqdiff);
         transmit(audio_freq);
      }   
      cycle = 0;
      Tx_last_mod_time = millis();
      last_audio_freq=audio_freq;
         
    }
    not_TX_first = 1;
    Tx_last_time = millis();
  }
  else if (millis()-Tx_last_time > 50) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receive. 
    Tx_Start = 0;
    cycle = 0;
    _INFO("TX-\n");
    receive();
    return;
  }  
  USBAudioRead();
}

void receiving() {
  USBAudioRead();  // read in the USB Audio buffer (myRawBuffer) to check the transmitting
  if (USBAudio_read) {
      Tx_Start = 1;
      not_TX_first = 0;
      _INFO("TX+ Tx_Start\n");
      return;
/*      
    }
*/    
  }
  freqChange();
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    USBAudioWrite(rx_adc, rx_adc);
  }
}

/*----------
  This procedure changes the transmission frequency if a frequency change has been detected
 */
void transmit(int64_t freq){
  
  /*--------
    If in receive mode then place it in transmission mode
   */ 
  if (Tx_Status==0){
  
#ifdef ADX-rp2040  
    digitalWrite(RXSW,LOW);   //Disable receiver
#else
    _INFO(" setting Tx_Status=%d\n",Tx_Status);
    digitalWrite(RXSW,HIGH);
#endif        

#ifdef ADX-rp2040
    digitalWrite(TX,HIGH);   //Turn TX LED on
#else
    showTX();                //Set RGB LED as red
#endif //ADX-rp2040

    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK2, 0);   //BFO. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    
    Tx_Status=1;
    _INFO("TX+\n");
  /*----------------------------------
    stop ADC collection
  */
    adc_run(false);                         //stop ADC free running

    return;
  }
  si5351.set_freq((RF_freq*100ULL + freq), SI5351_CLK0);  
 
}

/*-------------
  Place the transceiver in receiver mode and change the frequency accordingly
 */
void receive(){

  _INFO("TX_Status=%d\n",Tx_Status);
  if (Tx_Status != 0) {

     si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
     si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);

     si5351.set_freq((RF_freq-BFO_freq)*100ULL, SI5351_CLK1);
     
     si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
     si5351.output_enable(SI5351_CLK2, 1);   //BFO osc. on

  
     Tx_Status=0;
     _INFO("TX-\n"); 
  }

/*--------
  Turn TX led off and enable receiver
*/

#ifdef ADX-rp2040
     digitalWrite(TX,LOW);
     digitalWrite(RXSW,HIGH);
#else
     _INFO("showRX() Rx RF Freq=%ul\n",RF_freq);
     showRX();               //Show RGB led as blue (receiving mode)
     digitalWrite(RXSW,LOW);
#endif  //ADX-rp2040

  /*-----------
    Initialize the USB incoming queue
  */
  for (int i = 0; i < SAMPLESIZE; i++) {
    monodata[i] = 0;
  } 
  
  /*------------
    initialization of ADC and the data write counter
  */
  pcCounter=0;
  nBytes=0;

  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running

}

/*-------------
  This is a frequency changing procedure, placing the Si5351 clock values
  (not migrated yet)
*/  
void freqChange(){
  
  /*
  if (digitalRead(pin_SW)==LOW){
    delay(100);
    if  (digitalRead(pin_SW)==LOW){
      C_freq++;
      if  (C_freq >= N_FREQ){
        C_freq = 0;
      }
    }
    RF_freq = Freq_table[C_freq];
    si5351.set_freq(RF_freq*100, SI5351_CLK1);
    //NEOPIXEL LED change
    pixels.setPixelColor(0, colors[C_freq]);
    pixels.show();
    
    delay(100);
    adc_fifo_drain ();
    adc_offset = adc();
  }
  */

}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                             CAT management
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                       Support functions                                                    *
//                       (mostly original from ADX_UnO except few debug messages)                             *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

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
#ifdef CAT
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                     CAT Support functions                                                  *
//                partial implementation of the Kenwood TS2000 protocol (115200 8N2).                         *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void CAT_warning() {

#ifdef ADX-rp2040
      if (millis()-CATT1 >=CAT_WARN1 && CATT2 !=0 && cat_stat != 0) {

       flipLED=!flipLED;
       digitalWrite(WSPR, flipLED); 
       digitalWrite(FT8, flipLED);
       CATT1=millis();

       if (millis()-CATT2 >= CAT_WARN2) {

          CATT1=0;
          CATT2=0;
          digitalWrite(WSPR, LOW); 
          digitalWrite(FT8, LOW);

       } 
      }
#endif //ADX-rp2040

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
    _INFO("malformed command, ignored\n");
    return false;
  }

  if (strcmp(cmd,"FA")==0) {  
      if (strcmp(arg,"") != 0) {
        unsigned long fx=strtol(arg, &q, 10);
        uint16_t bx=freq2band(fx);
        if (bx==0) return true;
        Band_slot=band2slot(bx);
        band=bx;
        RF_freq=fx;
        si5351_setFreq(RF_freq);

        adc_fifo_drain ();
        adc_offset = adc();


        //si5351.set_freq(RF_freq * 100ULL, SI5351_CLK0);
        //si5351.set_freq(freq * 100ULL, SI5351_CLK1);
        
        CATT1=millis();
        CATT2=millis();
        receive();
      }    
      uint32_t fa=RF_freq;
      String sent = "FA" // Return 11 digit frequency in Hz.  
          + String("00000000000").substring(0,11-(String(fa).length()))   
          + String(fa) + ";";     
      strcpy(r,sent.c_str());
      return true;
  }
  

  if (strcmp(cmd,"PS")==0) {
      strcpy(r,"PS1;");
      return true;
  }

  if (strcmp(cmd,"TX")==0)  {   
      strcpy(r,"TX0;");
      transmit(AF_TONE);
      return true;
  } 

  if (strcmp(cmd,"RX")==0) {  
    strcpy(r,"RX0;");
    receive();
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

  if (strcmp(cmd,"IF")==0) {

      if (Tx_Status == 1) {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String((long int)RF_freq).length()))   
                  + String((long int)RF_freq) + "0000" + "+" + "00000" + "0" + "0" + "0" + "00" + "12" + "0000000;"; 
          strcpy(r,sent.c_str());        
      } else {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String((long int)RF_freq).length()))   
                  + String((long int)RF_freq) + "0000" + "+" + "00000" + "0" + "0" + "0" + "00" + "02" + "0000000;"; 
          strcpy(r,sent.c_str());
      } 
      return true;
  }


  if (strcmp(cmd,"MD")==0) {  
      strcpy(r,"MD2;");
      return true;
  }
  strcpy(r,"ID019;");   //dummy answer trying not to disrupt the CAT protocol flow
  _INFO("***ERROR*** Entry(%s) nor processed response(%s)\n",c,r);
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
int nread=Serial.available();
if (nread > 0){
   if (cat_stat == 0) {
       cat_stat=1;

#ifdef ADX-rp2040
       digitalWrite(WSPR, LOW); 
       digitalWrite(FT8, LOW);
       digitalWrite(JS8, HIGH);
       digitalWrite(FT4, HIGH);
#endif //ADX-rp2040

       receive();
   }
} else {
  return;
}

/*-----------
  Too small as a packet, perhaps fragmentation is occuring
*/
if (nread < 3) { 
  return;
}

/*-----------
  Read the serial port buffer
*/
int rc=Serial.readBytes(CATbuf,nread);
if (rc <= 1) {
  return;
}
CATbuf[rc]=0x0;
/*------------
  Look after spurious contents
*/
int k=strlen(serialBuf);
for (int i=0;i<strlen(CATbuf);i++){
    char c=CATbuf[i];
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

if (strcmp(serialBuf,"TX;ID;") == 0) {

    transmit(AF_TONE);
    strcpy(CATResp,"ID019;");
    Serial.print(CATResp);
    _INFO("CAT Command(%s) len(%d)\n",CATResp,strlen(CATResp));

    strcpy(CATResp,"");
    strcpy(serialBuf,"");      
    return;

}

/*-------------------
  More oddities, now with the receiving part
*/
    if (strcmp(serialBuf,"RX;ID;") == 0) {      
       receive();
       strcpy(CATResp,"ID019;");
       Serial.print(CATResp);
       _INFO("CAT Command(%s) len(%d)\n",CATResp,strlen(CATResp));

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
            Serial.print(CATResp);
            _INFO("CAT Command(%s) len(%d)\n",CATResp,strlen(CATResp));
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
                Serial.print(CATResp);
                _INFO("CAT Command(%s) len(%d)\n",CATResp,strlen(CATResp));
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
       Serial.print(resp);      
       _INFO("CAT Command(%s) len(%d)\n",resp,strlen(resp));

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
#endif //CAT
/*--------------
  Entry point for cat
*/
void cat() {

#ifdef CAT
  CAT_check();
#endif //CAT
}
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of CAT Protocol handler                                        *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                             end of RX/TX management cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


