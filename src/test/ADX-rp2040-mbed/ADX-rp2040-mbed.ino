//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022,2023
//
// Version 3.0
// This code relies heavily on the great work from Hitoshi, JE1RAV at the QP-7C_RP2040 transceiver and
// his generous sharing of insights and code leading to this solution.
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
// This is experimental code trying to port the ADX-rp2040 code to the Arduino IDE mbed core in order 
// to implement the link with WSJT-X thru an USB audio port
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*********************************************************************************************************
//* Based on ADX-rp2040 by Pedro Colla LU7DZ (2022)
//* Originally ported from ADX_UnO_V1.3 by Barb Asuroglu (WB2CBA)
//*********************************************************************************************************
//*
//* Code excerpts from different sources
//*
//* originally from ft8_lib by Karlis Goba (YL3JG), great library and the only one beyond WSJT-X itself
//* excerpts taken from pi_ft8_xcvr by Godwin Duan (AA1GD) 2021
//* excerpts taken from Orange_Thunder by Pedro Colla (LU7DID/LU7DZ) 2018
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
// Arduino Mbed OS RP2040 boards
// Board: "Raspberry Pi Pico"
//
// The firmware has been developed with a std Raspberry Pi Pico version using the Arduino IDE mbed core
//*****************************************************************************************************
// Arduino "Wire.h" I2C library         (built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library    (built-into arduino ide)
// To be installed using the Arduino IDE Library Manager
// Etherkit Si5351
// SI5351       (https://github.com/etherkit/Si5351Arduino) Library by Jason Mildrum (NT7S) 
// TFT_eSPI     (https://github.com/Bodmer/TFT_eSPI) Library by Bodmer
// TFT_eWidget  (https://github.com/Bodmer/TFT_eWidget) Library by Bodmer
// MDNS_Generic (https://github.com/khoih-prog/MDNS_Generic) Library by Khoi Hoang.
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

//*********************************************************************************************************

#include "PluggableUSBAudio.h"


/*------------------------------------------------------
 *   Internal clock handling
 */
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
time_t now;
char timestr[12];
#include "ADX-rp2040.h"


uint64_t RF_freq;   // RF frequency (Hz)
int C_freq = 0;  //FREQ_x: In this case, FREQ_0 is selected as the initial frequency.
int Tx_Status = 0; //0=RX, 1=TX
int Tx_Start = 0;  //0=RX, 1=TX
int not_TX_first = 0;
uint32_t Tx_last_mod_time;
uint32_t Tx_last_time;

//Audio signal frequency determination
int16_t mono_prev=0;  
int16_t mono_preprev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
int32_t cycle_frequency[34];


USBAudio audio(true, 44100, 2, 44100, 2);

int16_t monodata[24];
uint16_t pcCounter=0;
uint16_t nBytes=0;
uint8_t myRawBuffer[96]; //24 sampling (= 0.5 ms at 48000 Hz sampling) data sent from PC are received.
int16_t pcBuffer16[48];  //24 sampling date are written to PC in one packet.
int16_t USB_read=0;

static uint8_t buf[128];
char hi[256];
int i=0;

void USB_UAC() {
 // initialization of monodata[]
 for (int i = 0; i < 24; i++) {
   monodata[i] = 0;
 }
}

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



void setup() {
  for (int i = 0; i<sizeof(buf); i++) {
    buf[i] = 128 * sin(i);
  }
  Serial.begin(115200);
  Serial.flush();
  _INFO("Test ADX-rp2040-mbed\n");

 //USB Audio initialization
  USB_UAC();
  _INFO("Transceiver ready\n");

}

void loop() {
/*
  audio.write(buf, sizeof(buf));
  i++;
  if (i>10000) {
     i=0;
     Serial.println("Loop()");
  }
*/
  if (Tx_Start==0) {
     receiving();
  } else {
     transmitting();
  }

}

void transmitting(){
  int64_t audio_freq;
  if (USB_read) {
    for (int i=0;i<24;i++){
      int16_t mono = monodata[i];
      if ((mono_prev < 0) && (mono >= 0)) {
        if ((mono == 0) && (((float)mono_prev * 1.8 - (float)mono_preprev < 0.0) || ((float)mono_prev * 2.02 - (float)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
          if (Tx_Start==0) _INFO("TX- signal drop\n");
          Tx_Start = 0;
          
          break;
        }
        int16_t difference = mono - mono_prev;
        // x=0付近のsin関数をテーラー展開の第1項まで(y=xで近似）
        float delta = (float)mono_prev / (float)difference;
       
        float period = (1.0 + delta_prev) + (float)sampling - delta;
        audio_freq = 44100*100.0/period; // in 0.01Hz    
        if ((audio_freq>20000) && (audio_freq<300000)){
          cycle_frequency[cycle]=audio_freq;
          //_INFO("TX=%ld Hz\n",cycle_frequency[cycle]);
          cycle++;
        }
        delta_prev = delta;
        sampling=0;
        mono_preprev = mono_prev;
        mono_prev = mono;     
      }
      else if ((not_TX_first == 1) && (mono_prev == 0) && (mono == 0)) {        //Detect non-transmission
        Tx_Start = 0;
        //_INFO("TX- No-transmission detected\n");
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
      _INFO("audio_freq=%ld Hz\n",audio_freq);
      transmit(audio_freq);
      cycle = 0;
      Tx_last_mod_time = millis();
    }
    not_TX_first = 1;
    Tx_last_time = millis();
  }
  else if (millis()-Tx_last_time > 50) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receive. 
    Tx_Start = 0;
    _INFO("TX-\n");
    cycle = 0;
    receive();
    return;
  }  
  USBread();
}

void receiving() {
  USBread();  // read in the USB Audio buffer (myRawBuffer) to check the transmitting
  if (USB_read) {
    Tx_Start = 1;
    not_TX_first = 0;
    //_INFO("TX+\n");
    return;
  }
/*  
  freqChange();
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    rp.USBwrite(rx_adc, rx_adc);
  }
*/  
}

void transmit(int64_t freq){
  if (Tx_Status==0){
    /*
    digitalWrite(pin_RX,0);   //RX off
    digitalWrite(pin_TX,1);   //TX on
    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    Tx_Status=1;
    digitalWrite(pin_RED, 0);
    digitalWrite(pin_GREEN, 1);
    //digitalWrite(pin_BLUE, 1);

    adc_run(false);                         //stop ADC free running
    */

    _INFO("TX On\n");
    Tx_Status=1;
    return;
  }
  //_INFO("RX On\n");
  /*
  si5351.set_freq((RF_freq*100 + freq), SI5351_CLK0);  
  */
}

void receive(){
  /*
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.set_freq(RF_freq*100, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  */
  //_INFO("TX Off\n");
  Tx_Status=0;
  /*
  digitalWrite(pin_RED, 1);
  digitalWrite(pin_GREEN, 0);
  //digitalWrite(pin_BLUE, 1);
  */

  // initialization of monodata[]
  for (int i = 0; i < 24; i++) {
    monodata[i] = 0;
  } 
  /*
  // initialization of ADC and the data write counter
  rp.pcCounter=0;
  rp.nBytes=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
  */
}

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

