//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                            Pedro (Pedro Colla) - LU7DZ - 2022,2023
//
//                                         Version 3.0
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
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
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
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "ADX_rp2040-mbed"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  "1.0"
#define BUILD     "01"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#include "hardware/adc.h"
#include "PluggableUSBAudio.h"
#include "si5351.h"
#include "Wire.h"
#include "ADX-rp2040.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//.                Master clock
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
Si5351 si5351;
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//.                Transceiver Frequency management memory areas
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
uint16_t bands[NBANDS] = {40,30,20,10};
uint64_t freqs[NBANDS] = {7074000,10136000,14074000,28074000};

uint16_t band=40;
uint16_t band_slot=0;
//uint64_t freq=7074000;
uint64_t RF_freq=7074000;   // RF frequency (Hz)

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

/*-------------------------------*
   Sample data
*/
/*
extern uint16_t adc_v1;
extern uint16_t adc_v2;
extern uint32_t adc_t1;
extern uint32_t adc_t2;
*/

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

USBAudio audio(true, SAMPLE_RATE, 2, SAMPLE_RATE, 2);

int16_t monodata[24];
uint16_t pcCounter=0;
uint16_t nBytes=0;
uint8_t myRawBuffer[96]; //24 sampling (= 0.5 ms at 48000 Hz sampling) data sent from PC are received.
int16_t pcBuffer16[48];  //24 sampling date are written to PC in one packet.
int16_t USB_read=0;
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

if (s >= 0 && s <= (NBANDS-1)) {
   return bands[s];  
}
return 40;
  
}
/*---------------
  given the band return the frequency associated with it from freqs[]
*/  
uint64_t band2freq(uint16_t b) {

if (b >160 || b < 10) {
   return 40;
}

for (int i=0;i<NBANDS;i++) {
   if (bands[i]==b) {
       return freqs[i];
   }  
}
return freqs[0];

}
/*------------------------
  given the band return the slot
*/  
uint16_t band2slot(uint16_t b) {

if (b >160 || b < 10) {
   return 0;
}

for (int i=0;i<NBANDS;i++) {
   if (bands[i]==b) {
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

return 40;

}
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


  setLEDbyband(band);
  _INFO("Set slot[%d] band[%d] mts freq=%ul Hz\n",band_slot,band,RF_freq);

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

   if (cat_stat == 1) return;

   if (getSW(UP)==LOW) {
      while(getSW(UP)==LOW);
      band_slot=(band_slot+1);
      if (band_slot>NBANDS-1) { 
        band_slot=0;
      }
      setSlot(band_slot);         
   }

   if (getSW(DOWN)==LOW) {
      while(getSW(DOWN)==LOW);
      int new_slot=band_slot-1;
      if (new_slot<0) {
         band_slot = NBANDS-1;
      }  else {
         band_slot=new_slot;
      }
      setSlot(band_slot);
   }

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
 for (int i = 0; i < 24; i++) {
   monodata[i] = 0;
 }
}
/*-------------
  UAC2 USB data read
 */
void USBread() {
  USB_read = audio.read(myRawBuffer, sizeof(myRawBuffer));
  if (USB_read) {
    int16_t *lessRawBuffer = (int16_t *)myRawBuffer;
    for (int i = 0; i < 24; i++) {
      // the left value;
      int16_t outL = *lessRawBuffer;
      lessRawBuffer++;
      // the right value
      int16_t outR = *lessRawBuffer;
      lessRawBuffer++;
      //mono value
      int16_t mono = (outL + outR) / 2;
      monodata[i] = mono;
    }
  }
}
/*-------------
  UAC2 data write
 */
void USBwrite(int16_t left,int16_t right) {
  if(nBytes>95){
    uint8_t *pcBuffer =  (uint8_t *)pcBuffer16;
    audio.write(pcBuffer, 96);
    pcCounter =0;
    nBytes =0;
  }
  pcBuffer16[pcCounter]=left;
  pcCounter++;
  nBytes+=2;
  pcBuffer16[pcCounter]=right;
  pcCounter++;
  nBytes+=2;
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                Si5351 sub-system
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*---------
  set Si5351 frequency
*/
void si5351_setFreq(uint64_t f) {

  si5351.set_freq(f*100ULL, SI5351_CLK0);  //for TX
  si5351.set_freq((f-BFO_freq)*100ULL, SI5351_CLK1);  //for RX
  si5351.set_freq(BFO_freq*100ULL, SI5351_CLK2);  //for BFO

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


  _INFO("Master clock sub-system Ok!\n");
}  
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                Board management
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void init_IO() {

  pinMode(A0, INPUT);            //ADC input pin

  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(TXSW, INPUT_PULLUP);

  pinMode(TX, OUTPUT);

  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);

  pinMode(RX, OUTPUT);

/*--------------------
     Initialize starting mode (RX on,TX off, all LED off)
  */
  digitalWrite(RX, HIGH);
  digitalWrite(TX, LOW);
  digitalWrite(WSPR,LOW);
  digitalWrite(FT8,LOW);
  digitalWrite(FT4,LOW);
  digitalWrite(JS8,LOW);

  _INFO("Board I/O initialized\n");

}
/*-------------
  Set a given LED set by the slot
*/  
void setLEDbyslot(uint16_t s) {

  for (int i=0;i<4;i++) {
      if (i==s) {
         digitalWrite(7-i,HIGH);
      } else {
         digitalWrite(7-i,LOW);
      }
  }

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

  _INFO("ADX-rp2040-mbed digital transceiver v(%s) build(%s)\n",VERSION,BUILD);

/*-----------
  Init I/O
*/
  init_IO();

/*-----------
  Set initial band, slot, frequency and set LED
*/
  setSlot(band_slot);

//ADC initialize ----- 
  adc_setup();


/*-----------
  Initialize Si5351
 */
  si5351_init();

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
void transmitting(){

  int64_t audio_freq;
  /*-------
    Is there any data on the USB in queue?
   */
  if (USB_read) {    //USB_read=true there is data on the USB Queue
    /*-------
      Collect the data from the USB buffer
     */
    for (int i=0;i<24;i++){
      
      int16_t mono = monodata[i];
      
      if ((mono_prev < 0) && (mono >= 0)) {    // Check for a zero crossing
        
        /*------------
          Check if there is a sudden drop to zero
         */
        if ((mono == 0) && (((float)mono_prev * 1.8 - (float)mono_preprev < 0.0) || ((float)mono_prev * 2.02 - (float)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
           Tx_Start=0;
           last_audio_freq=0;
           break;
        }
        /*-------------
          Check for differences between current USB value and previous, compute the difference                  
        */
        int16_t difference = mono - mono_prev;

        /*------ (Original from Hitoshi-san code)
        // x=0付近のsin関数をテーラー展開の第1項まで(y=xで近似）
        // Sin(x) function near x=0 can be approximated with the the first term of Taylor series (y=x approximation)

        Compute the audio frequency from the incoming signal
        */
        float delta = (float)mono_prev / (float)difference;
        float period = (1.0 + delta_prev) + (float)sampling - delta;
        audio_freq = SAMPLE_RATE*100.0/period; // in 0.01Hz    

        if ((audio_freq>FSK_MIN) && (audio_freq<FSK_MAX)){
          cycle_frequency[cycle]=audio_freq;
          cycle++;
        }
        /*------
          Store previous values
         */
        delta_prev = delta;
        sampling=0;
        mono_preprev = mono_prev;
        mono_prev = mono;     

      } else 
        /*-------
          Detect non-transmission
         */
        if ((not_TX_first == 1) && (mono_prev == 0) && (mono == 0)) {        //Detect non-transmission
           Tx_Start=0;
           last_audio_freq=0;
           break;
        }
       else {
        /*-------
          Prepare for next sample
         */
        sampling++;
        mono_preprev = mono_prev;
        mono_prev = mono;
      }
    }
    //End of USB data collection and processing cycle

    /*-----------
      If reach here because of the signal break then Tx_Status == 0 and the
      transceiver needs to be placed in receive mode
     */
    if (Tx_Start == 0){
      cycle = 0;
      receive();
      return;     // Gotta be out of Dodge City
    }
    /*------------
      This is a cycle throttle meassurement where even if a frequency change has been detected no
      change will be propagated to the Si5351 unless a minimum of FSK_THRESHOLD (msec) has been elapsed
      avoiding cluttering the I2C bus with noise
    */  
    if ((cycle > 0) && (millis() - Tx_last_mod_time > FSK_THRESHOLD)){ 
      audio_freq = 0;
      
      for (int i=0;i<cycle;i++){
        audio_freq += cycle_frequency[i];
      }
      
      audio_freq = audio_freq / cycle;

      long unsigned freqdiff=abs((long int)audio_freq-(long int)last_audio_freq);
      
      if (freqdiff > FSK_MIN_CHANGE) {
         _INFO("FSK=%ld Hz (diff=%lu)\n",(long int)audio_freq,freqdiff);
         transmit(audio_freq);
      }   
      cycle = 0;
      Tx_last_mod_time = millis();
      last_audio_freq = audio_freq;

    }
    /*------------
      Update first occurrence flag and timers for next cycle
     */
    not_TX_first = 1;
    Tx_last_time = millis();
  
  } else      //end of if(USB_Read) 
    /*----------------
      USBAudio data has not been received for more than 50 msec during transmission, VOX is turned off
     */  
    if (millis()-Tx_last_time > FSK_TOUT) { 
       Tx_Start = 0;
       cycle = 0;
       last_audio_freq=0;
       receive();
       return;
    }   
  
  
  /*------------------
    Read USB Data again
   */

  USBread();
}

/*--------------
  Main receiving control cycle
 */
void receiving() {
  /*---------
    Read the USB incoming queue
  */
  USBread();
  /*---------
    If while in receive mode data is present in the USB buffer then a transmission cycle
    must be started, it's assumed this is a digital link with the transmitting program
    thus it will be perfectly silent if no transmission is made
  */

  if (USB_read) {

    /*---------------
      Place the transceiver in transmit mode
     */
    Tx_Start = 1;
    not_TX_first = 0;
    return;
  }
  freqChange();
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)

  /*-------------------
    write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  */  
  for (int i=0;i<6;i++){
    USBwrite(rx_adc, rx_adc);
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
  
    digitalWrite(RX,LOW);   //Disable receiver
    digitalWrite(TX,HIGH);   //Turn TX LED on
    
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
  
  if (Tx_Status != 0) {

  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);
  si5351.set_freq((RF_freq-BFO_freq)*100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  si5351.output_enable(SI5351_CLK2, 1);   //BFO osc. on

/*--------
  Turn TX led off and enable receiver
*/

  digitalWrite(TX,LOW);
  digitalWrite(RX,HIGH);
  
  Tx_Status=0;
  _INFO("TX-\n");
  
  }
  /*-----------
    Initialize the USB incoming queue
  */
  for (int i = 0; i < 24; i++) {
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
        uint16_t band_slot=band2slot(bx);
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
       digitalWrite(WSPR, LOW); 
       digitalWrite(FT8, LOW);
       digitalWrite(JS8, HIGH);
       digitalWrite(FT4, HIGH);
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


