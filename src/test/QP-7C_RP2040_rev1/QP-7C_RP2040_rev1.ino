/*
 * Copyright (C) 2023- Hitoshi Kawaji <je1rav@gmail.com>
 * 
 * QP-7C_RP2040.ino.
 * 
 * QP-7C_RP2040.ino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * QP-7C_RP2040.ino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
 
#include "Arduino.h"
//#include "mbed.h"
//#include "USBAudio.h"
#include "PluggableUSBAudio.h"
//#include <Wire.h>
#include "hardware/adc.h"

#include <stdint.h>
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"
//#include "hardware/uart.h"

#include <si5351.h>


/*-------------------------------------------------
 * Macro expansions
 */
#define digitalWrite(x,y) gpio_put(x,y)
#define digitalRead(x)  gpio_get(x)

#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP (byte)0


#define AUDIOSAMPLING 48000  // USB Audio sampling frequency

#define N_FREQ 2 // number of using RF frequencies　(<= 7)
#define FREQ_0 7041000 // RF frequency in Hz
#define FREQ_1 7074000 // in Hz
//#define FREQ_2 7074000 // in Hz
//#define FREQ_3 7074000 // in Hz
//#define FREQ_4 7074000 // in Hz
//#define FREQ_5 7074000 // in Hz
//#define FREQ_6 7074000 // in Hz
extern uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}

#define pin_RX 27 //pin for RX switch (D1,output)
#define pin_TX 28 //pin for TX switch (D2,output)
#define pin_SW 3 //pin for freq change switch (D10,input)
#define pin_RED 17 //pin for Red LED (output)
#define pin_GREEN 16 //pin for GREEN LED (output)
#define pin_BLUE 25 //pin for BLUE LED (output) 
#define pin_LED_POWER 11 //pin for NEOPIXEL LED power (output)
#define pin_LED 12 //pin for NEOPIXEL LED (output)

#include <si5351.h>  //"Etherkit Si5351"
Si5351 si5351;

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, pin_LED);
uint8_t color = 0;
#define BRIGHTNESS 5  //max 255
uint32_t colors[] = {pixels.Color(BRIGHTNESS, 0, 0), pixels.Color(0, BRIGHTNESS, 0), pixels.Color(0, 0, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, 0), pixels.Color(BRIGHTNESS, 0, BRIGHTNESS), pixels.Color(0, BRIGHTNESS, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS)};
  
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


//ADC offset for recieving
int16_t adc_offset = 0;   

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

void setup() {
  //Digital pin setting ----- 
  pinMode(A0, INPUT); //ADC input pin
  pinMode(pin_SW, INPUT_PULLUP); //SW (freq. change)
  pinMode(pin_RX, OUTPUT); //RX →　High, TX →　Low (for RX switch)
  pinMode(pin_TX, OUTPUT); //TX →　High, RX →　Low (for Driver switch)
  pinMode(pin_RED, OUTPUT); //On →　LOW (for RED LED)
  pinMode(pin_GREEN, OUTPUT); //On →　LOW (for GREEN LED)
  pinMode(pin_BLUE, OUTPUT); //On →　LOW (for BLUE LED)
  pinMode(pin_LED_POWER, OUTPUT); //NEOPIXEL LED

  //I2c initialization-----  
  Wire.begin();

  //ADC initialize ----- 
  adc_init();
  adc_select_input(0);                        //ADC input pin A0
  adc_run(true);                              //start ADC free running
  adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
  adc_fifo_setup(true,false,0,false,false);   // fifo
  
  //USB Audio initialization
  rp.USB_UAC();

  //si5351 initialization-----  
  int32_t cal_factor = -11800;
  RF_freq = Freq_table[C_freq];
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);  //for TX
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK1);  //for RX
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK1, 0);

  //NEOPIXEL LED  initialization-----
  digitalWrite(pin_LED_POWER, HIGH);  //NEOPIXEL LED ON
  pixels.begin();  // initialize the NEOPIXEL
  pixels.setPixelColor(0, colors[C_freq]);
  pixels.show();

  //transceiver initialization-----
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  digitalWrite(pin_RED, HIGH);
  digitalWrite(pin_GREEN, LOW); //Green LED ON
  digitalWrite(pin_BLUE, HIGH);

  //read the DC offset value of ADC input (about 0.6V)----- 
  delay(500);
  adc_fifo_drain ();
  adc_offset = adc();

  Serial.begin(115200);
  Serial.flush();
  Serial.println("prueba de validación anárquica\n");  
}

void loop() {
  if (Tx_Start==0) receiving();
  else transmitting();
  Serial.println("looping here\n");
}

void transmitting(){
  int64_t audio_freq;
  if (rp.USB_read) {
    for (int i=0;i<24;i++){
      int16_t mono = rp.monodata[i];
      if ((mono_prev < 0) && (mono >= 0)) {
        if ((mono == 0) && (((float)mono_prev * 1.8 - (float)mono_preprev < 0.0) || ((float)mono_prev * 2.02 - (float)mono_preprev > 0.0))) {    //Detect the sudden drop to zero due to the end of transmission
          Tx_Start = 0;
          break;
        }
        int16_t difference = mono - mono_prev;
        // x=0付近のsin関数をテーラー展開の第1項まで(y=xで近似）
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
        Tx_Start = 0;
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
      transmit(audio_freq);
      cycle = 0;
      Tx_last_mod_time = millis();
    }
    not_TX_first = 1;
    Tx_last_time = millis();
  }
  else if (millis()-Tx_last_time > 50) {     // If USBaudio data is not received for more than 50 ms during transmission, the system moves to receive. 
    Tx_Start = 0;
    cycle = 0;
    receive();
    return;
  }  
  rp.USBread();
}

void receiving() {
  rp.USBread();  // read in the USB Audio buffer (myRawBuffer) to check the transmitting
  if (rp.USB_read) {
    Tx_Start = 1;
    not_TX_first = 0;
    return;
  }
  freqChange();
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    rp.USBwrite(rx_adc, rx_adc);
  }
}

void transmit(int64_t freq){
  if (Tx_Status==0){
    digitalWrite(pin_RX,0);   //RX off
    digitalWrite(pin_TX,1);   //TX on
    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    Tx_Status=1;
    digitalWrite(pin_RED, 0);
    digitalWrite(pin_GREEN, 1);
    //digitalWrite(pin_BLUE, 1);

    adc_run(false);                         //stop ADC free running
  }
  si5351.set_freq((RF_freq*100 + freq), SI5351_CLK0);  
}

void receive(){
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.set_freq(RF_freq*100, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  Tx_Status=0;
  digitalWrite(pin_RED, 1);
  digitalWrite(pin_GREEN, 0);
  //digitalWrite(pin_BLUE, 1);
    
  // initialization of monodata[]
  for (int i = 0; i < 24; i++) {
    rp.monodata[i] = 0;
  } 
  
  // initialization of ADC and the data write counter
  rp.pcCounter=0;
  rp.nBytes=0;
  adc_fifo_drain ();                     //initialization of adc fifo
  adc_run(true);                         //start ADC free running
}

void freqChange(){
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
}

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

void rp2040Audio::USB_UAC() {
 audio= new USBAudio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);
 // initialization of monodata[]
 for (int i = 0; i < 24; i++) {
   monodata[i] = 0;
 }
}

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
