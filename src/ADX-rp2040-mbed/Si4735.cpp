#include <Arduino.h>
#include "ADX-rp2040.h"
#ifdef RX_SI473X
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * RDX_Si4735.ino                                                                              *
 * The table below shows the Si4735 and RASPBERRY PICO connections modified for the RDX project*                                                                                           *   
 * | ---------------| ---------- |                                                             *
 * | Si4735 pin     |  PICO Pin  |                                                             *
 * | ---------------| ---------- |                                                             *
 * | RESET (pin 15) |     GP16   |                                                             *
 * | SDIO (pin 18)  |     GP0    |                                                             *
 * | CLK (pin 17)   |     GP1    |                                                             *
 * | ---------------| ---------- |                                                             *
 * PU2CLR Si47XX API documentation: https://pu2clr.github.io/SI4735/extras/apidoc/html/        *
 * RDX project documentation: http://www.github.com/lu7did/RDX-rp2040                          *
 *                                                                                             *
 * Released by the public domain By Ricardo Lima Caratti, Nov 2019.                            *
 * Released to the public domain By Dr. Pedro E. Colla, Apr 2023.                              *
 *---------------------------------------------------------------------------------------------*
  Copyright (c) 2019 by Ricardo Lima Caratti. All rights reserved.                     
  This file is part of the Arduino Pico core board library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *---------------------------------------------------------------------------------------------*
 * Adaptation and integration with RDX project by Dr. Pedro E. Colla (LU7DZ) 2022,2023         *
 *                                                                                             *
 * | ---------------| ---------- |                                                             *
 * | Si4735 pin     |   RDX Pin  |                                                             *
 * | ---------------| ---------- |                                                             *
 * | RESET (pin 15) |     GP1    |                                                             *
 * | SDIO (pin 18)  |     GP16   | (same than Si5351 sharing the I2C bus)                      *
 * | CLK (pin 17)   |     GP17   | (same than Si5351 sharing the I2C bus)                      *
 * | ---------------| ---------- |                                                             *
 
 *---------------------------------------------------------------------------------------------*
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include <SI4735.h>
#include "patch_init.h" // SSB patch for whole SSBRX initialization string

const uint16_t size_content = sizeof ssb_patch_content; // see patch_init.h
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                       Global varibles                                                       *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool led=false;
bool disableAgc=false;
bool avc_en=true;

SI4735 rx;

int currentBFO = 975;
uint16_t currentFrequency;
uint8_t currentStep = 1;
uint8_t currentBFOStep = 25;
uint8_t bandwidthIdx = 3;   //Was 2 for 3.0 KHz, now it is 3 for 4.0 KHz
const char *bandwidth[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
uint8_t currentAGCAtt = 0;  // currentAGCAtt == 0)
uint8_t si473x_rssi   = 0;
uint32_t rssitimer = 0;
/*------------------------------------------------------------------------------------------
 * set the Si473x chip frequency 
  *-----------------------------------------------------------------------------------------*/
void SI473x_setFrequency(int s) {


  uint16_t b=Bands[s];
  uint16_t i=Band2Idx(b);

  _INFO("<min> slot[%d][1]=%d -- <max>slot[%d][2]=%d - <curr> slot[%d][0]=%d\n",s,(uint16_t)(slot[i][1]/1000),s,(uint16_t)(slot[i][2]/1000),s,(uint16_t)(slot[i][0]/1000));

  uint16_t minimumFreq=(uint16_t)(slot[i][1]/1000);
  uint16_t maximumFreq=(uint16_t)(slot[i][2]/1000);
  uint16_t currentFreq=(uint16_t)(slot[i][0]/1000);

  uint16_t currentStep=1;
  uint16_t currentSSB=USB;
  _INFO("currentStep=%d currentSSB=%d\n",currentStep,currentSSB);
   
  rx.setSSB(minimumFreq,maximumFreq,currentFreq,currentStep,currentSSB);
  delay(10);

  currentFrequency = rx.getCurrentFrequency();
  _INFO("Current Frequency established as %d\n",currentFrequency);
  
  String freqDisplay;
  freqDisplay = String((float)currentFrequency / 1000, 3);
  _INFO("RF_Freq(%llu) Slot(%d) Band(%d) Index(%d) currentFreq(%d) min(%d) max(%d) Frequency set to %s. KHz\n",RF_freq,Band_slot,b,i,currentFreq,minimumFreq,maximumFreq,freqDisplay.c_str());

}
/*------------------------------------------------------------------------------------------
 * setup the Si473x sub-system, find out the Si473x chip I2C address and connect to it
 * then initialize the frequency of operation and other parameters needed.
  *-----------------------------------------------------------------------------------------*/
void SI473x_Setup()
{
  digitalWrite(RESET_SI473X, HIGH);
  delay(10);

  _INFO("Loading Si473x library (c) PU2CLR in SSB mode reset(%d)\n",int16_t(RESET_SI473X));
  delay(100);
  
  /*-------------------------------------
    get the Si473x I2C buss address
   */
  int16_t si473xAddr = rx.getDeviceI2CAddress(RESET_SI473X);
  if ( si473xAddr == 0 ) {
    _INFO("Si473X chip not found, processing is blocked!\n");
    while (1);
  } else {
    _INFO("Si473X chip found at address 0x%X\n",si473xAddr);
  }

  rx.setup(RESET_SI473X, AM_FUNCTION);
  delay(10);

  
  _INFO("SSB patch is loading...\n");
  long et1 = millis();
  rx.setI2CFastModeCustom(500000); // Increase the transfer I2C speed
  rx.loadPatch(ssb_patch_content, size_content); // It is a legacy function. See loadCompressedPatch 
  rx.setI2CFastModeCustom(100000); 
  long et2 = millis();
 
  _INFO("SSB patch loaded successfully (%ld msec)\n",(et2-et1));
  delay(10);
  
  rx.setTuneFrequencyAntennaCapacitor(1); // Set antenna tuning capacitor for SW.

  _INFO("Calling setFrequency Slot=%d\n",Band_slot);
  SI473x_setFrequency(Band_slot);
  
  currentStep = 1;

  delay(10); 
  rx.setFrequencyStep(currentStep);  
  _INFO("set FrequencyStep as %d\n",currentStep);

/*----------------
  Set the Audio bandwidth to 4 KHz
*/
  rx.setSSBAudioBandwidth(bandwidthIdx);
  _INFO("set AudioBandwidth=%d\n",bandwidthIdx);
  
  rx.setSSBSidebandCutoffFilter(1);
  _INFO("SSB cut off filter set\n");
  

  rx.setSSBBfo(currentBFO);  
  _INFO("set BFO as %d\n",currentBFO);
  delay(100);
/*-----------------
  Switch on/off AGC disableAGC=1 disable, AGC Index=0. Minimum attenuation (max gain)
*/  
  rx.setAutomaticGainControl(disableAgc, currentAGCAtt);  
  _INFO("set AGC as %d==%d\n",disableAgc,currentAGCAtt);
  delay(100);

  avc_en=false;
  rx.setSSBAutomaticVolumeControl(avc_en);  
  _INFO("set automatic volume controlas %d\n",avc_en);
  delay(100);

/*------------------
  Maximum gain for automatic volume control on AM/SSB mode (12 to 90dB)
*/
  rx.setVolume(255);    //MV
  delay(10);
  _INFO("set automatic volume control as %d\n",getVolume());

  currentFrequency = rx.getCurrentFrequency();
  String freqDisplay;
  freqDisplay = String((float)currentFrequency / 1000, 3);
   _INFO("current frequency read from Si473x is %s\n",freqDisplay.c_str());
 
  SI473x_enabled=true;
  SI473x_Status(); //for test purpose only
 

} //SI473x_setup() end

/*-----------------------------------------
 * Show the current receiver status
 */
void SI473x_Status() {

if (!SI473x_enabled) {
   _INFO("Si473x device not available yet, request ignored\n");
   return;
}   

  rx.getAutomaticGainControl();
  rx.getCurrentReceivedSignalQuality();

  String bfo;
  if (currentBFO > 0)
    bfo = "+" + String(currentBFO);
  else
    bfo = String(currentBFO);

  String freqDisplay = getFrequency();

_INFO("|AGC:%s|LNA GAIN index:%d/%d|BW:%s KHz|S=%d|SNR %d|RSSI %d dBuV|Volume: %d|BFO %s|BFOStep: %d|freq=%s|Step: %d|\n", \
       (rx.isAgcEnabled() ? "AGC ON" : "AGC OFF"), \
        rx.getAgcGainIndex(), \
        currentAGCAtt, \
        bandwidth[bandwidthIdx], \
        getSignal(), \
        getSNR(), \
        getRSSI(), \
        getVolume(), \
        bfo.c_str(), \
        currentBFOStep, \
        freqDisplay.c_str(), \
        currentStep);
rx.setI2CFastModeCustom(100000); // Increase the transfer I2C speed
  
}
int getRSSI() {
    if (time_us_32()-rssitimer > 1000000) {
       rx.getCurrentReceivedSignalQuality();
       rssitimer=time_us_32();
    }
    return rx.getCurrentRSSI();

}

int getSignal() {
    int rssi
    
     = rx.getCurrentRSSI();
    int rssiAux = 0;
    if (rssi < 2) rssiAux = 4;
    else
       if (rssi < 4) rssiAux = 5;
       else
          if (rssi < 12) rssiAux = 6;
          else
             if (rssi < 25) rssiAux = 7;
             else
                if (rssi < 50) rssiAux = 8;
                else
                  rssiAux = 9;
   return rssiAux;
}
String getFrequency() {
   int currentFrequency = rx.getCurrentFrequency();
   String freqDisplay;
   freqDisplay = String((float)currentFrequency / 1000, 3);
   return freqDisplay;
}
int getSNR() {
   int snr=rx.getCurrentSNR();   //Needs to previously call getRSSI to have a fresh reading of the signal level
   return snr;   
}
int getVolume() {   
   int Vol=rx.getCurrentVolume();
   return Vol;
 
}
void setVolume() {
  rx.setVolume(255);  
}

#endif //RX_SI473X
