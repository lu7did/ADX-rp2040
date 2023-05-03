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
// The USB soundcard implementation (UAC2) has been largely extracted from the code Hitoshi Kawaji (JE1RAV) from his
// transceiver implementation (http://github.com/je1rav/QP-7C_RP2040)
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

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "ADX_mbed"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  1.0
#define BUILD     01
/*-------------------------------------------------*/

#include <si5351.h>
#include "Wire.h"
#include "Arduino.h"
#include "mbed.h"
#include "USBAudio.h"
#include "hardware/adc.h"
#include <stdint.h>
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "ADX-mbed.h"


/*------------------------------------------------------
   Main variables
*/
char hi[256];
//uint32_t codefreq = 0;
//uint32_t prevfreq = 0;

/*--------------------------------------------
  Structures to held the time
*/
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t now;
time_t t_ofs;
char timestr[12];


/*----------------------------------------------------------------------------
  ADX declarations
*/
int mode=4;                          //mode=1 (WSPR),2 (JS8), 3 (FT4) and 4 (FT8)
unsigned long freq=7074000;          //master frequency in Hz
unsigned long freq1=freq;
int TX_State = 0;                    //TX_State=0 (RX) 1(TX)

/*----------------------------------------------------------------------------
  Frequencies of different modes
*/
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

int cal_factor=0;

int Band1 = 40; // Band 1 // These are default bands. Feel free to swap with yours
int Band2 = 30; // Band 2
int Band3 = 20; // Band 3
int Band4 = 10; // Band 4 //*RP2040* changed to 10m from 17m
MbedI2C i2c(I2C_SDA,I2C_SCL);
Si5351 si5351;
UART uartSerial(UART_TX,UART_RX);

//uint64_t RF_freq;   // RF frequency (Hz)

/*
int C_freq = 0;  //FREQ_x: In this case, FREQ_0 is selected as the initial frequency.
int Tx_Status = 0; //0=RX, 1=TX
*/

int not_TX_first = 0;
int64_t BFO_freq = 0;
uint32_t Tx_last_mod_time;
uint32_t Tx_last_time;

//Audio signal frequency determination

int16_t mono_prev=0;  
int16_t mono_preprev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
int32_t cycle_frequency[34];


//ADC offset for receiving
int16_t adc_offset = 0;   

int cat_stat = 0;
int CAT_mode = 2;   

#ifdef CAT

/*-------------------------------------------------------------------------------------------
  CAT Processing loop
*/
#define CATCMD_SIZE          18

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


//USB Audio modified from pdmAudio.h in https://github.com/tierneytim/Pico-USB-audio
class rp2040Audio {
  public:
    //Constructor
    rp2040Audio();
    void USB_UAC();
    void USBread();
    void USBwrite(int16_t left,int16_t right);
    int16_t monodata[24];
    uint16_t pcCounter=0;
    uint16_t nBytes=0;
    bool USB_read;
  private:
    USBAudio* audio;
    uint8_t myRawBuffer[96]; //24 sampling (= 0.5 ms at 48000 Hz sampling) data sent from PC are received.
    int16_t pcBuffer16[48];  //24 sampling date are written to PC in one packet.
 };

rp2040Audio rp;

/*-----------------------------------------------------------------------------------------------------------
  transmitting
  while signal is present measure frequency by a zero cross method as described by Hans Summers in the 
  QCX manual. Once detected the frecuency is applied as a shift in the frequency of the Si5351 to produce
  the modulation.
*/  
void transmitting(){
  int64_t audio_freq;
  if (rp.USB_read) {
    for (int i=0;i<24;i++){
      int16_t mono = rp.monodata[i];
      if ((mono_prev < 0) && (mono >= 0)) {
        if ((mono == 0) && (((float)mono_prev * 1.8 - (float)mono_preprev < 0.0) || ((float)mono_prev * 2.02 - (float)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
          TX_State = 0;
          break;
        }
        int16_t difference = mono - mono_prev;
        float delta = (float)mono_prev / (float)difference;
       
        float period = (1.0 + delta_prev) + (float)sampling - delta;
        audio_freq = AUDIOSAMPLING*100.0/period; // in 0.01Hz    
        if ((audio_freq>20000) && (audio_freq<300000)){
          cycle_frequency[cycle]=audio_freq;
          cycle++;
        }
        delta_prev = delta;
        sampling=0;
        mono_preprev = mono_prev;
        mono_prev = mono;     
      }
      else if ((not_TX_first == 1) && (mono_prev == 0) && (mono == 0)) {        //Detect non-transmission
        TX_State = 0;
        break;
      }
      else {
        sampling++;
        mono_preprev = mono_prev;
        mono_prev = mono;
      }
    }
    if (TX_State == 0){
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
      transmit(audio_freq);
      cycle = 0;
      Tx_last_mod_time = millis();
    }
    not_TX_first = 1;
    Tx_last_time = millis();
  }
  else if (millis()-Tx_last_time > 50) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receive. 
    TX_State = 0;
    cycle = 0;
    receive();
    return;
  }  
  rp.USBread();
}

/*------------------------------------------------------------------------------------------------------------
  receiving()
  the incoming signal is sampled at the ADC port and sent to the USB host
*/  
void receiving() {
  rp.USBread();  // read in the USB Audio buffer (myRawBuffer) to check the transmitting
  
  if (rp.USB_read) {
    TX_State = 1;
    not_TX_first = 0;
    return;
  }

  //freqChange();    WARNING CHECK LATER
  
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    rp.USBwrite(rx_adc, rx_adc);
  }
}

/*-------------------------------------------------------------------------------------------
  transmit
  turns on the transmitter and changes the frequency
  output of the Si5351 in transmitting is thru CLK0
*/  
void transmit(int64_t affreq){
  if( TX_State==0){

    digitalWrite(RX,0);   //RX off
    digitalWrite(TX,1);   //TX on

    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on

    TX_State=1;
    adc_run(false);                         //stop ADC free running
  }
  si5351.set_freq((freq*100ULL + affreq), SI5351_CLK0);     //*** WARNING *** Review correct mix
}
/*----------------------------------------------------------------------------------------------
  receive
  turns off the transmitter and return to the base frequency
*/
void receive(){
  digitalWrite(TX,0);  //TX off
  digitalWrite(RX,1);  //RX on

  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.set_freq((freq-BFO_freq)*100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  TX_State=0;
  for (int i = 0; i < 24; i++) {
    rp.monodata[i] = 0;
  } 
  
  // initialization of ADC and the data write counter
  rp.pcCounter=0;
  rp.nBytes=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
}
/*---------------------------------------------------------------------------
  freqChange()
  changes the Si5351 frequency
*/  
/*
void freqChange(){
  if (digitalRead(pin_SW)==LOW){
    delay(100);
    if  (digitalRead(pin_SW)==LOW){
      C_freq++;
      if  (C_freq >= N_FREQ){
        C_freq = 0;
      }
    }
    freq = Freq_table[C_freq];
    si5351.set_freq((freq-BFO_freq)*100ULL, SI5351_CLK1);
    delay(100);
    adc_fifo_drain ();
    adc_offset = adc();
  }
}
*/
int16_t adc() {
  int16_t adc = 0;
  for (int i=0;i<24;i++){             // 192kHz/24 = 8kHz
    adc += adc_fifo_get_blocking();   // read from ADC fifo
  }  
  return adc;
}

//USB Audio modified from pdmRP2040.cpp in https://github.com/tierneytim/Pico-USB-audio
rp2040Audio::rp2040Audio() {
}

/*----------------------------------------------------------------------------
  implement a method for receiving thru the USB port
*/  
void rp2040Audio::USB_UAC() {
 audio= new USBAudio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);

 // initialization of monodata[]
 for (int i = 0; i < 24; i++) {
   monodata[i] = 0;
 }
}
/*------------------------------------------------------------------------------
  implement a method for transmitting thru the USB port
*/  
void rp2040Audio::USBread() {
  USB_read = audio->read(myRawBuffer, sizeof(myRawBuffer));
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

void rp2040Audio::USBwrite(int16_t left,int16_t right) {
  if(nBytes>95){
    uint8_t *pcBuffer =  (uint8_t *)pcBuffer16;
    audio->write(pcBuffer, 96);
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
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 *                                      Setup                                                  *
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void setup()
{

_SERIAL.begin(115200);
_SERIAL.setTimeout(4);
while(!_SERIAL);
delay(50);
_SERIAL.flush();

/*--- QC ---*/
 //Digital pin setting ----- 
 /*
  pinMode(A0, INPUT); //ADC input pin
  pinMode(pin_SW, INPUT_PULLUP); //SW (freq. change)
  pinMode(pin_RX, OUTPUT); //RX →　High, TX →　Low (for RX switch)
  pinMode(pin_TX, OUTPUT); //TX →　High, RX →　Low (for Driver switch)
  pinMode(pin_RED, OUTPUT); //On →　LOW (for RED LED)
  pinMode(pin_GREEN, OUTPUT); //On →　LOW (for GREEN LED)
  pinMode(pin_BLUE, OUTPUT); //On →　LOW (for BLUE LED)
  pinMode(pin_LED_POWER, OUTPUT); //NEOPIXEL LED
*/
  //I2c initialization-----  
  i2c.begin();

  //ADC initialize ----- 
  adc_init();
  adc_select_input(0);                        //ADC input pin A0
  adc_run(true);                              //start ADC free running
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo
  
  //USB Audio initialization
  rp.USB_UAC();

  //si5351 initialization-----  
  int32_t cal_factor = 0;
  freq = 7074000;
   
  /*-----
     Set direction of input ports
  */
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(TXSW, INPUT_PULLUP);

  
  /*---
     Set output ports
  */
  pinMode(RX, OUTPUT);
  pinMode(TX, OUTPUT);
  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);



  /*
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(freq*100ULL, SI5351_CLK0);  //for TX
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.set_freq((freq-BFO_freq)*100ULL, SI5351_CLK1);  //for RX. **** WARNING ***
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK1, 0);
*/

 //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  //----------------------------------------------------------------------------------
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on

  _INFO("si5351 clock initialization completed\n");

/*-----------------------------------------------------------------------
   transceiver initialization
*/
Band_assign();
Freq_assign();
Mode_assign();

/*
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  digitalWrite(pin_RED, HIGH);
  digitalWrite(pin_GREEN, LOW); //Green LED ON
  digitalWrite(pin_BLUE, HIGH);
*/
  //read the DC offset value of ADC input (about 0.6V)----- 

  delay(500);
  adc_fifo_drain ();
  adc_offset = adc();


  //i2c.begin();
  _INFO("I/O setup completed\n");

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
/*
      addr = 40;                   //Save current mode in EEPROM
      EEPROM.put(addr, mode);

#ifdef RP2040
      EEPROM.commit();   //rp2040 doesn't have any EEPROM, the core library emulates EEPROM on flash memory, but it requires a commit() to set
#endif //RP2040          
*/

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
/*
      addr = 40;
      EEPROM.put(addr, mode);

#ifdef RP2040
      EEPROM.commit(); //rp2040 doesn't have an EEPROM, the core library emulates EEPROM on flash memory and requires a commit() to set values permanently
#endif //RP2040          
*/
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
     This is the audio frequency counting algorithm based on digital data received from the USB port
     The USB soundcard implementation (UAC2) has been largely extracted from the code Hitoshi Kawaji (JE1RAV) from his
     transceiver implementation (http://github.com/je1rav/QP-7C_RP2040)
   *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

  if (TX_State==0) {
    receiving();
  } else { 
    transmitting();
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

#ifdef EXCLUDE
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
          _INFO("VOX turned ON\n");
          continue;
        }
        /*-----------------------------------------------------
           Avoid producing jitter by changing the frequency
           by less than 4 Hz.
        */
        if (abs(int(codefreq - prevfreq)) >= FSK_ERROR) {
          unsigned long fx = ((unsigned long)(freq + codefreq)) * 100ULL;
          _INFO("freq=%ld codefreq=%ld si5351(f)=%lu\n",freq, codefreq, fx);
          si5351.set_freq(fx, SI5351_CLK0);
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
        _INFO("VOX turned OFF\n");
      }
    }
#endif //EXCLUDE

#ifdef CAT  

    CAT_check();
    CAT_warning();
  
#endif //CAT
}
#ifdef CAT
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
    _INFO("malformed command, ignored\n");
    return false;
  }

  if (strcmp(cmd,"FA")==0) {  
      if (strcmp(arg,"") != 0) {
        unsigned long fx=strtol(arg, &q, 10);
        freq=fx;        
        freq1=fx;
        si5351.set_freq(freq * 100ULL, SI5351_CLK0);
        si5351.set_freq(freq * 100ULL, SI5351_CLK1);
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

  if (strcmp(cmd,"IF")==0) {
      if (TX_State == 1) {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String(freq).length()))   
                  + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "1" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";"; 
          strcpy(r,sent.c_str());        
      } else {  
          String sent = "IF" // Return 11 digit frequency in Hz.  
                  + String("00000000000").substring(0,11-(String(freq).length()))   
                  + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "0" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";"; 
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
_INFO("CAT Command(%s) len(%d)\n",serialBuf,strlen(serialBuf));

if (strcmp(serialBuf,"TX;ID;") == 0) {

    setTX(HIGH);
    strcpy(CATResp,"ID019;");
    _CAT.print(CATResp);
    _INFO("CAT Command(%s) len(%d)\n",CATResp,strlen(CATResp));

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
            _CAT.print(CATResp);
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
                _CAT.print(CATResp);
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
       _CAT.print(resp);      
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
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of CAT Protocol handler                                        *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#endif //CAT

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of loop()                                                      *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


//************************************[ MODE Assign ]**********************************

void Mode_assign() {


  if ( mode == 1) {
    freq = F_WSPR;
    digitalWrite(WSPR, HIGH);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);

  }

  if ( mode == 2) {
    freq = F_JS8;
    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, HIGH);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, LOW);

  }

  if ( mode == 3) {
    freq = F_FT4;

    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, HIGH);
    digitalWrite(FT8, LOW);
  }
  if ( mode == 4) {
    freq = F_FT8;

    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, LOW);
    digitalWrite(FT4, LOW);
    digitalWrite(FT8, HIGH);
  }
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
