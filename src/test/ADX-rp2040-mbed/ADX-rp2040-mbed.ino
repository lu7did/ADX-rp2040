//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ADX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022,2023
//
//                                         Version 3.0
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//* Based on ADX-rp2040 by Pedro Colla LU7DZ (2022)
//* Originally ported from ADX_UnO_V1.3 by Barb Asuroglu (WB2CBA)
//
// This is experimental code trying to port the ADX-rp2040 code to the Arduino IDE mbed core in order 
// to implement the link with WSJT-X thru an USB audio port
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
// Etherkit Si5351
// SI5351       (https://github.com/etherkit/Si5351Arduino) Library by Jason Mildrum (NT7S) 
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

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

#include "PluggableUSBAudio.h"
#include "ADX-rp2040.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//.                Transceiver Frequency management memory areas
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
uint64_t RF_freq;   // RF frequency (Hz)
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
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                               setup cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup() {
  /*----------
    Serial port initialization
   */
  Serial.begin(115200);
  Serial.flush();
  _INFO("Test ADX-rp2040-mbed\n");

 //USB Audio initialization
  USB_UAC();
  _INFO("Transceiver USB Sub-system ready\n");
  

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                               loop cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void loop() {

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
/*  THIS IS CODE NOT MIGRATED YET RELATED TO THE READING OF RX DATA FROM THE ADC PORT
    AT THIS POINT THE FIRMWARE IS STILL A TRANSMIT ONE
  freqChange();
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    USBwrite(rx_adc, rx_adc);
  }
*/  
}

/*----------
  This procedure changes the transmission frequency if a frequency change has been detected
 */
void transmit(int64_t freq){
  
  /*--------
    If in receive mode then place it in transmission mode
   */ 
  if (Tx_Status==0){
  
    /*
    digitalWrite(pin_RX,0);   //RX off
    digitalWrite(pin_TX,1);   //TX on
    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    */
    Tx_Status=1;
    _INFO("TX+\n");
    /*
    digitalWrite(pin_RED, 0);
    digitalWrite(pin_GREEN, 1);
    //digitalWrite(pin_BLUE, 1);

    adc_run(false);                         //stop ADC free running
    */

    return;
  }
  // CODE YET TO BE INTEGRATED TO CHANGE THE Si5351 CLK0 port
  /*
  si5351.set_freq((RF_freq*100 + freq), SI5351_CLK0);  
  */
}

/*-------------
  Place the transceiver in receiver mode and change the frequency accordingly
 */
void receive(){
  
  if (Tx_Status != 0) {
  /*
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.set_freq(RF_freq*100, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  */
  
  Tx_Status=0;
  _INFO("TX-\n");
  
  /*
  digitalWrite(pin_RED, 1);
  digitalWrite(pin_GREEN, 0);
  //digitalWrite(pin_BLUE, 1);
  */
  }
  /*-----------
    Initialize the USB incoming queue
  */
  for (int i = 0; i < 24; i++) {
    monodata[i] = 0;
  } 
  
  /*
  // initialization of ADC and the data write counter
  pcCounter=0;
  nBytes=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
  */
  
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
//*                                             end of RX/TX management cycle
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


