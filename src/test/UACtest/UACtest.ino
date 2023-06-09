/*
* USBAudio example
*
* @code
*/
#include "mbed.h"
#include "USBAudio.h"
/*
* // Audio loopback example use:
* // 1. Select "Mbed Audio" as your sound device
* // 2. Play a song or audio file
* // 3. Record the output using a program such as Audacity
*
*/
#define SERIAL     1
#define AUDIOSAMPLING 48000  // USB Audio sampling frequency

static uint8_t buf[128];
char hi[256];
//USBAudio audio(true, 44100, 2, 44100, 2);
UART mySerial(12, 13);
#define _Serial Serial

//USB Audio modified from pdmAudio.h in https://github.com/tierneytim/Pico-USB-audio
class rp2040Audio {
  public:
    //Constructor
    rp2040Audio();
    void USB_UAC();
    void USBread();
    void USBwrite(int16_t left,int16_t right);
    void USBout();
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
//ADC offset for recieving
int16_t adc_offset = 0;   


void setup() {

//audio= new USBAudio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);
//USB Audio initialization
//  rp.USB_UAC();
pinMode(A0, INPUT); //ADC input pin

//ADC initialize ----- 
adc_init();
adc_select_input(0);                        //ADC input pin A0
adc_run(true);                              //start ADC free running
adc_set_clkdiv(249.0);                      // 192kHz sampling  (48000 / (249.0 +1) = 192)
adc_fifo_setup(true,false,0,false,false);   // fifo

//read the DC offset value of ADC input (about 0.6V)----- 
delay(500);
adc_fifo_drain ();
adc_offset = adc();


#ifdef SERIAL
  _Serial.begin(115200);
  //while(!_Serial);
  _Serial.flush();
  sprintf(hi,"Looping audio\r\n");
  _Serial.print(hi);
#endif //SERIAL

for (int i = 0; i<sizeof(buf); i++) {
    buf[i] = 128 * sin(i);
}

}
void loop() {
/*
  int16_t rx_adc = adc() - adc_offset; //read ADC data (8kHz sampling)
  // write the same 6 stereo data to PC for 48kHz sampling (up-sampling: 8kHz x 6 = 48 kHz)
  for (int i=0;i<6;i++){
    rp.USBwrite(rx_adc, rx_adc);
  }
*/
  
  //audio.write(buf, sizeof(buf));
  rp.USBout();
  #ifdef SERIAL
  _Serial.println("looping by");
  #endif //
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
void rp2040Audio::USBout() {
    audio->write(buf, 128);
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

