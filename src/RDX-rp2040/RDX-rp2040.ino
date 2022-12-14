//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              RDX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 1.0
//
// This is the implementation of a rp2040 firmware for a monoband, self-contained FT8 transceiver based on
// the ADX hardware architecture, using Karliss Goba's ft8lib and leveraging on several other projects
// 
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
// Created with Arduino IDE using the Arduino-Pico core created by Earle F. Philhower, III available
// at https://github.com/earlephilhower
// Check for installation and configuration instructions at
// https://www.upesy.com/blogs/tutorials/install-raspberry-pi-pico-on-arduino-ide-software
//
//                                     *******************************************
//                                     *                Warning                  *
//                                     *******************************************
//
// This firmware is meant to be used with an ADX board where the Arduino Nano or Arduino Uno processor has been replaced
// by a raspberry pi pico board plus addicional voltage and signal conditioning circuits, please see the host site
// https://github.com/lu7did/RDX-rp2040 for construction details and further comments.
//
// This firmware is meant to be compiled using the latest Arduino IDE environment with the following parameters
//
// Board: "Raspberry Pi Pico W"
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
// The firmware has been developed with a Raspberry Pi Pico W version but it should work with a regular Raspberry Pi Pico
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide)
// SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*****************************************************************************************************
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*************************************[ LICENCE]*********************************************
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
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// The following Si5351 VFO calibration procedure has been extracted from the ADX-rp2040
// firmware which in turns has been derived from ADX-UnO_V1.3. The original procedure has
// been developed by Barb (WB2CBA) as part of his firmware code.
//*****************[ SI5351 VFO MANUAL CALIBRATION PROCEDUR***********************************
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
/*-----------------------------------------------
   ADX-rp2040 includes, headers and definition
*/

#include "RDX-rp2040.h"
#include "crc.h"
#include "constants.h"
#include "pack.h"
#include "encode.h"
#include "decode_ft8.h"
#include "gen_ft8.h"

#include "tx_ft8.h"
#include "rx_ft8.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"

#include "pico/multicore.h"
#include "hardware/irq.h"

/*------------------------------------------------------
   Main variables
*/
char hi[128];
/*----------------------
 * state variables
 */
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

/*-----------------------
 * Station data
 */
char my_callsign[16];
char my_grid[8];

/*-----------------------
 * Program identification
 */
char programname[12];
char version[6];
char build[6];

/*-----------------------
 * Wifi support
 */
char ip[16];

/*------------------------------------------------------
 *   Internal clock handling
 */
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
time_t now;
char timestr[12];

/*-----------------------
 * FT8 decoding
 */
uint8_t  num_decoded = 0;
uint32_t tdecode = 0;
uint8_t  nTry = 0;
uint8_t  maxTx = 6;
uint8_t  maxTry = 5;
uint8_t  nRx = 0;
uint8_t  nTx = 0;
uint8_t  state = 0;
bool     qwait=true;
bool     triggerCQ=false;
bool     triggerCALL=false;
bool     send = false;
bool     justSent = false; //must recieve right after sending

bool autosend = false;

/*---------------------
 * Definitions related to the autocalibration function
 */
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
int      pwm_slice;
uint32_t f_hi;
uint32_t fclk     = 0;
int32_t  error    = 0;


/*-------------------------------------------------------
   ft8 definitions
*/

message_info CurrentStation;
UserSendSelection sendChoices;
message_info message_list[kMax_decoded_messages]; // probably needs to be memset cleared before each decode

/*
bool fCQ = false;
int  nCQ = 0;
int  qCQ = 5;
bool cq_only = false;
*/

uint64_t config_us;
uint64_t fine_offset_us = 0; //in us
int16_t signal_for_processing[num_samples_processed] = {0};
uint32_t handler_max_time = 0;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                             Global Variables for ADX                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
uint32_t val;
int temp;
uint32_t val_EE;
int addr = 0;


/*------------------------------------------
 * Band and frequency information
 */
const uint16_t Bands[BANDS] = {40, 30, 20, 10};     // Band1,Band2,Band3,Band4 (initial setup)
const unsigned long slot[MAXBAND] = { 3573000UL,       //80m [0]
                                      5357000UL,       //60m [1]
                                      7074000UL,       //40m [2]
                                     10136000UL,       //30m [3]
                                     14074000UL,       //20m [4]
                                     18100000UL,       //17m [5]
                                     21074000UL,       //15m [6]
                                     24915000UL,       //12m [7]
                                     28074000UL};      //10m [8]
int Band_slot = 0;
int Band = 0;

int Band1 = Bands[0]; // Band 1 // These are default bands. Feel free to swap with yours
int Band2 = Bands[1]; // Band 2
int Band3 = Bands[2]; // Band 3
int Band4 = Bands[3]; // Band 4 //*RP2040* changed to 10m from 17m

unsigned long freq = 7074000UL;

int32_t cal_factor;
int TX_State = 0;

int UP_State;
int DOWN_State;
int TXSW_State;
int Bdly = 250;

Si5351 si5351;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  Time sync, wait till second 0,15,30 or 45                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
bool timeWait() {

  now = time(0) - t_ofs;
  gmtime_r(&now, &timeinfo);
  if (timeinfo.tm_sec % 15 == 0) {
    return true;
  }
  return false;

}
/*--------------------------------------------------------------------------------------------
 * This is a callback handler which is called when the fft is completed for each ADC sample
 * (1000 samples at 6000 Hz), this can be useful to perform housekeeping such as to update
 * a waterfall display.
 */
void fftCallBack() {
  
    tft_updatewaterfall(magint);
    tft_checktouch();
    checkButton();
    tft_run();

}
/*---------------------------------------------------------------------------------------------
 * This is a callback handler which is called at the end of each decoding cycle
 */
void endCallBack() {
    tft_endoftime();
    checkButton();
    tft_run();
}
/*----------------------------------------------------------------------------------------------
 * This is the callback handler called while sending FT8 tones
 * *WARNING*
 * At this point calling this callback ruins the elaboration of ft8 tones, therefore it's not
 * being called
 */
void idleCallBack() {
    tft_checktouch();
    checkButton();
    tft_run();
}
/*--------------------------------------------------------------------------------------------
 * This is a callback handler which is called when the ft8 decoding process has identified
 * a valid reception of a line. The index points to the entry on message_list where the
 * newly received message is.
 */
void qsoCallBack(int i) {
  tft_run();
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  ft8 processing                                                                          *
//*  setup and handle the ft8 processing, this is actually the orchestation of the ft8lib    *
//*  functions by Karliss Goba                                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup_ft8() {

  /*===================================================================*
   * WARNING                                                           *
   * the original firmware overclock the processor, the initial version* 
   * of this firmware won't                                            *
   *===================================================================*/
  /*-------------------------------------------------------------------*/
  //*overclocking the processor
  //*133MHz default, 250MHz is safe at 1.1V and for flash
  //*if using clock > 290MHz, increase voltage and add flash divider
  //*see https://raspberrypi.github.io/pico-sdk-doxygen/vreg_8h.html
  //*vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
  //*@@@ set_sys_clock_khz(250000,true);
  /*-------------------------------------------------------------------*/

  /*------
     setup the ADC processing
  */
  setup_adc();

  /*------
     make the Hanning window for fft work
  */
  make_window();

  /*------
     Establish a callback handler to be called at the end
     of each ADC sample (to update the waterfall)
  */

  fftReady=fftCallBack;
  fftEnd=endCallBack;
  qsoReady=qsoCallBack;
  txIdle=idleCallBack;
  
  /*-------
   * Wait to settle
   */
  sleep_ms(500);

} //end of the ft8 setup

/*---------------------------------------------------------------------------------------------
   ft8bot
   this is a finite state automata enabled by autosend=true and controlled by
   maxTx  (every how many cycles an autonomous CQ attempt is made)
   maxTry (how many retries are allowed
   Two distinctive topology arcs are followed depending on whether a CQ was made (0->1->2->3)
   or answered (0->5->6->7) by the bot.
   A standard FT8 QSO cycle is assumed.
   This bot is created with the purpose to automate a manual GUI later in the development

   GUI operation

   in order to perform a CQ the finite state machine needs to be entered with state=1 and
   nTry=maxTry+1 and will start calling CQ after the next receiving cycle

   in order to answer a CQ the finite state machine needs to be entered with state=5 and
   the UserSendSelection populated with the data of the station to be answered,also nTry
   must be maxTry+1

 *                                  *********************************
 *                                  *         Warning               *
 *                                  *********************************

   (1) In order to properly operate without producing QRM on the channel the clock must be in sync
   (2) The bot should not be left unattended
   (3) Under normal operation the autosend=false
  ---------------------------------------------------------------------------------------------*/
bool ft8bot(message_info *CurrentStation, UserSendSelection *sendChoices, message_info message_list[]) {

/*----------------
 * State 0
 * Define if a CQ is going to be sent, this can happen because it's in auto mode
 * and just completed a receiving cycle or because it has been forced to call CQ
 */
  _INFOLIST("%s processing state=%d\n",__func__,state);
  
  if (state == 0  && !justSent && (autosend || triggerCQ)  ) {   //State 0 - Just completed a reception exploring if trigger a call (CQ Cycle)
    if (nTx < maxTx && !triggerCQ) {  //if autosend then transmit a CQ every maxTx to avoid overwhelming the channel
      nTx++;
      _INFOLIST("%s state(%d) Tx(%d) autosend active waiting\n", __func__, state, nTx);
    } else {
      _INFOLIST("%s state(%d) Calling CQ\n", __func__, state);
      state = 1;
      triggerCQ=false;
      sendChoices->call_cq = true;
      strcpy(CurrentStation->station_callsign, "");
      strcpy(CurrentStation->grid_square, "");
      CurrentStation->af_frequency=1500;
      CurrentStation->self_rx_snr=0;
      CurrentStation->qsowindow=getQSOwindow();
      nTry = 0;
      nTx=0;
      return true;
  }
  }

/*-----------------
 * State 0 (bis)
 * This is while in idle, handle the special case of a triggerCALL
 * flag set
 */
  if (state == 0 && !justSent && triggerCALL) {
       _INFOLIST("%s activating a response from triggerCALL station(%s) grid(%s) SNR(%d) af(%d)\n",__func__,call_station_callsign,call_grid_square,call_self_rx_snr,call_af_frequency);
       state=5;                               //Synchro with FSM as if the CQ was answered automatically
       triggerCALL=false;
       sendChoices->send_grid = true;
       /*-----------
        * recover data from pointed QSO line
        */
       strcpy(CurrentStation->station_callsign, call_station_callsign);
       strcpy(CurrentStation->grid_square, call_grid_square);
       CurrentStation->self_rx_snr=call_self_rx_snr;
       CurrentStation->af_frequency=call_af_frequency;
       CurrentStation->qsowindow=call_qsowindow;
       /*-----------
        * 
        */
       nTry = 0;
       nTx=0;
       return true;
  }
/*-----------------
 * State 0
 * Still idle, so if there is some message received check if a CQ has been sent by 
 * somebody, if it's in auto mode then it might be wise to answer, but if there
 * are no decoded messages and it's at state 0 (idle) then it's a moot point to evaluate
 */
  if (num_decoded == 0 && state == 0) {
      _INFOLIST("%s num_decoded=%d returning without action\n",__func__,num_decoded);
      return false;
  }

/*-----------------
 * State 0
 * Some stations were received, check if there is a CQ among some of them and if
 * in auto mode then evaluate to answer it
 */

  if (state == 0  && !justSent && autosend) {   //State 0 - Just completed a reception, check if somebody call CQ (Answering Cycle) if auto mode is enabled
    _INFOLIST("%s state(%d) Looking for CQ\n", __func__, state);
    for (int i = 0; i < num_decoded; i++) {
      if (message_list[i].type_cq) {
        _INFOLIST("%s state(%d) msg[%d]=%s CQ call\n", __func__, state, i, message_list[i].full_text);
        state = 5;
        strcpy(CurrentStation->station_callsign, message_list[i].station_callsign);
        strcpy(CurrentStation->grid_square, message_list[i].grid_square);
        CurrentStation->self_rx_snr=message_list[i].self_rx_snr;
        CurrentStation->af_frequency=message_list[i].af_frequency;
        CurrentStation->qsowindow=message_list[i].qsowindow;
        sendChoices->send_grid = true;
        nTry = 0;
        nTx=0;
        return true;
      }
    }
  }

/*-------------
 * State 1
 * In this state of the FSM a CQ has been sent, but it's yet to be answered
 * by somebody. So check if somebody answered to me with a valid frame.
 * If so, send the SNR to continue the QSO cycle. If not, send a CQ again until
 * the number of tries exceed the maximum, if so, reset the FSM to an idle state
 */

  if ((state == 1) && !justSent) {  //State 1  - (CQ Cycle) Check if somebody answer my call, repeat the CQ if not
    for (int i = 0; i < num_decoded; i++) {
      if (message_list[i].addressed_to_me) {
        _INFOLIST("%s state(%d) msg[%d]=%s Addressed to me Grid(%s) snr(%s)\n", __func__, state, i, message_list[i].full_text, message_list[i].grid_square,message_list[i].snr_report);
        state = 2;
        strcpy(CurrentStation->station_callsign, message_list[i].station_callsign);
        strcpy(CurrentStation->grid_square, message_list[i].grid_square);
        CurrentStation->self_rx_snr=message_list[i].self_rx_snr;   
        CurrentStation->qsowindow=message_list[i].qsowindow;
        sendChoices->send_snr = true;
        nTry = 0;
        nTx=0;
        return true;
      }
    }
    /*-----
     * Nobody answered, thus start another CQ cycle
     */
    nTry++;
    if (nTry<maxTry && state == 1) {
       _INFOLIST("%s no answer keep calling CQ nTry(%d)\n",__func__,nTry);
       sendChoices->call_cq = true;
       strcpy(CurrentStation->station_callsign, "");
       strcpy(CurrentStation->grid_square, "");
       CurrentStation->self_rx_snr=0;
       CurrentStation->qsowindow=0;
       nTx=0;     
       return true; 
    }
    /*-------
     * The number of tries exceeded the allowed, so reset the FSM
     */
    _INFOLIST("%s state(%d) CQ not answered, reset\n", __func__, state);
    state = 0;
    nTx = 0;
    nTry = 0;
    return false;
  }

/*--------------
 * State 2
 * The QSO cycle progress to the next stage, evaluate if the other station sent a R-xx (Rsnr report)
 * and if so complete the QSO cycle sending 73
 * If the number of tries exceeds tha allowed then reset the FSM and return to idle (lost QSO).
 * Otherwise retry with a SNR report.
 */
  if (state == 2 && !justSent) { // State 2 - (CQ Cycle) After an answer with a Grid was provided send the SNR
    for (int i = 0; i < num_decoded; i++) {
      if (message_list[i].addressed_to_me && message_list[i].type_Rsnr) {
        _INFOLIST("%s state(%d) msg[%d]=%s Addressed to me and R-NN\n", __func__, state, i, message_list[i].full_text);
        state = 3;
        sendChoices->send_RRR = true;
        nTry = 0;
        nTx=0;
        return true;
      }
    }
    /*-------------------------
     * Check for the number of retries performed
     * and reset if exceeded
     */
    if (nTry >= maxTry) {
      _INFOLIST("%s state(%d) retry exceeded, reset\n", __func__, state);
      state = 0;
      nTx = 0;
      nTry = 0;
      return false;
    }
    /*---------------------------
     * Retry the SNR message
     */
    _INFOLIST("%s state(%d) repeat SNR\n", __func__, state);
    sendChoices->send_snr = true;
    nTry++;
    return true;
  }

/*---------------
 * State 3
 * To continue the QSO check if a 73 or RR73 has been received, if so complete the QSO
 * Check for number of retries and reset the FSM if exceeded
 * If not persevere into sending a 73 message
 */
  if (state == 3 && !justSent) { // State 3 - (CQ Cycle) After an answer with the 73 finalize the cycle
    for (int i = 0; i < num_decoded; i++) {
      if (message_list[i].addressed_to_me && (message_list[i].type_73)) {
        _INFOLIST("%s state(%d) msg[%d]=%s Addressed to me and 73 or RR73\n", __func__, state, i, message_list[i].full_text);
        state = 4;
        sendChoices->send_73 = true;
        nTry = 12;
        nTx=0;
        return true;
      }
    }
    /*------------------------
     * Review the retries and
     * reset if exceeded
     */
    if (nTry >= maxTry) {
      _INFOLIST("%s state(%d) QSO finalized, reset\n", __func__, state);
      state = 0;
      nTx = 0;
      nTry = 0;
      return false;
    }
    /*------------------------
     * Send a 73 frame again
     */
    _INFOLIST("%s state(%d) repeat 73\n", __func__, state);
    sendChoices->send_73 = true;
    nTry++;
    nTx=0;
    return true;
  }

/*----------------------
 * State 5
 * This state means that We answered a CQ from another station and We're waiting
 * for her reply (a SNR report). If found reply with a RSNR report (R-xx) frame.
 * If not retry the Grid message to answer the original CQ until the number
 * of retries is exceeded.
 */
  if (state == 5 && !justSent) { // State 5 - (Answer Cycle) Somebody answer my grid with a SNR, respond back with RNNN (Rsnr)
    for (int i = 0; i < num_decoded; i++) {
      if (strcmp(message_list[i].station_callsign, CurrentStation->station_callsign) == 0 && message_list[i].addressed_to_me) {
        if (message_list[i].type_snr) {
          _INFOLIST("%s state(%d) msg[%d]=%s SNR detected answering RSNR\n", __func__, state, i, message_list[i].full_text);
          state = 6;
          sendChoices->send_Rsnr = true;
          nTry = 0;
          nTx=0;
          return true;
        }
        if (message_list[i].type_cq) { // State 5 - (Answer cycle) if the station is still calling the send the Grid again
          _INFOLIST("%s state(%d) msg[%d]=%s still CQ, repeat grid\n", __func__, state, i, message_list[i].full_text);
          sendChoices->send_grid = true;
          strcpy(CurrentStation->station_callsign, message_list[i].station_callsign);
          nTry++;
          return true;
        }
      }
    }
    /*----------------------------
     * Check the number of retries
     * and reset if exceeded
     */
    if (nTry >= maxTry) {
      _INFOLIST("%s state(%d) retry exceeded, reset\n", __func__, state);
      state = 0;
      nTx = 0;
      nTry = 0;
      return false;
    }

    /*------------------------------
     * send the GRID message again
     */
    _INFOLIST("%s state(%d) repeat grid\n", __func__, state);
    sendChoices->send_grid = true;
    nTry++;
    nTx=0;
    return true;
  }
  /*----------------------------
   * State 6
   * Continuing with the QSO cycle the other station should send a
   * RRR message which is replied with 73 to complete the QSO
   * If not received resend t RSNR message until the retries allowed
   * are exceeded.
   */
  if (state == 6 && !justSent) {
    for (int i = 0; i < num_decoded; i++) {
      if (strcmp(message_list[i].station_callsign, CurrentStation->station_callsign) == 0 && message_list[i].addressed_to_me) {
        if (message_list[i].type_RRR || message_list[i].type_RR73) {
          _INFOLIST("%s state(%d) msg[%d]=%s RRR or RR73 message sending 73\n", __func__, state, i, message_list[i].full_text);
          state = 7;
          sendChoices->send_73 = true;
          strcpy(CurrentStation->station_callsign, message_list[i].station_callsign);
          nTry = 12;
          nTx=0;
          return true;
        }
        
        if (message_list[i].type_snr) {
          _INFOLIST("%s state(%d) msg[%d]=%s Still SNR repeat RSNR\n", __func__, state, i, message_list[i].full_text);
          sendChoices->send_Rsnr = true;
          nTry++;
          return true;
        }
      }
    }
    if (nTry >= maxTry) {
      _INFOLIST("%s state(%d) retry exceeded, reset\n", __func__, state);
      state = 0;
      //tft_reset();
      nTx = 0;
      nTry = 0;
      return false;
    }

    _INFOLIST("%s state(%d) repeat RSNR\n", __func__, state);
    sendChoices->send_Rsnr = true;
    nTry++;
    nTx=0;
    return true;
  }

  return false;
}
/*------------------------
 * Check if there are any message type flagged to be sent
 */
bool isChoices ( UserSendSelection *sendChoices) {

   if(sendChoices->call_cq  || sendChoices->send_grid || sendChoices->send_snr || sendChoices->send_Rsnr || sendChoices->send_RRR || sendChoices->send_RR73 || sendChoices->send_73) {
     return true;    
   }
   return false;

}
/*--------------------------------------------
 * Reset all sending choices after transmit
 */
void resetChoices (  UserSendSelection *sendChoices ) {

  sendChoices->call_cq = false;
  sendChoices->send_grid = false;
  sendChoices->send_snr = false;
  sendChoices->send_Rsnr = false;
  sendChoices->send_RRR = false;
  sendChoices->send_RR73 = false;
  sendChoices->send_73 = false;

}
/*---------------------------------------------
 * get the FT( protocol QSO window
 * secs 00-14 and 30-44 are window 0 whilst
 * secs 15-29 and 45-59 are window 1
 * QSO frames heard within window 0 can be replied
 * on window 1 only and viceversa
 */
int getQSOwindow() {
    now = time(0) - t_ofs;
    gmtime_r(&now, &timeinfo);
    if ((timeinfo.tm_sec >=0 && timeinfo.tm_sec < 15) || (timeinfo.tm_sec >=30 && timeinfo.tm_sec < 45)) {
         return 0;
    }
    return 1;
}

/*---------------------------------------
   this is the main ft8 decoding cycle,
   called once per loop() execution
*/
void ft8_run() {

  char message[25] = {0};
  uint8_t tones[79] = {0};
  char msg[40];

  /****************************************
   *             TX Cycle                 *
   ****************************************/

  while (!timeWait()) {
     tft_checktouch();
     checkButton();
  }
  //_INFOLIST("%s send(%s) isChoices(%s) CurrentStationWindow(%d) QSOwindow(%d)\n",__func__,BOOL2CHAR(send),BOOL2CHAR(isChoices(&sendChoices)),CurrentStation.qsowindow,getQSOwindow());  
  if (send && isChoices(&sendChoices) && CurrentStation.qsowindow != getQSOwindow())
  {

    _INFOLIST("%s ------- TX w[%d]----------\n", __func__,getQSOwindow());
    /*---------------------------------------------------------------*
     *  If in autosend mode pick automatically the response message  *
     *  if not pick it manually                                      *
     *---------------------------------------------------------------*/
    uint32_t txstart = time_us_32();
    if (state != 0) {
      manual_gen_message(message, CurrentStation, sendChoices, my_callsign, my_grid);
      resetChoices(&sendChoices);
      _INFOLIST("%s manual_gen_message(%s)\n", __func__, message);
      uint16_t qso=2;
      int self_rx_snr=0;
      sprintf(msg,"%04d %4d %s", CurrentStation.af_frequency, self_rx_snr, message);

    /*---------------------------------------------------------------*
     * Post the message to be sent to the text area of the GUI       *
     *---------------------------------------------------------------*/
      char txt[8];
      strcpy(txt,"");
      uint16_t qsot=2;
      int qsowindow=getQSOwindow();
      tft_storeQSO(qsowindow,qsot,msg,CurrentStation.af_frequency,0,txt,txt);

    /*---------------------------------------------------------------*
     * If signaled just reset the state in order for this message to *
     * be the last sent.                                             *
     *---------------------------------------------------------------*/
      if (nTry >= 12) {
        state = 0;
        tft_endQSO();
      }
    }
    /*---------------------------------------------------------------*
     *  Now generate the ft8 message to be sent in terms of tone     *
     *  sequences                                                    *
     *---------------------------------------------------------------*/
    if (strcmp(message,"") != 0) {
       generate_ft8(message, tones);
    /*---------------------------------------------------------------*
       Send the tone sequences generated
      ---------------------------------------------------------------*/
       send_ft8(tones, freq, CurrentStation.af_frequency);
    }

    /*---------------------------------------------------------------*
       place the cycle in receiver mode and flag it completion
      ---------------------------------------------------------------*/
    send = false;
    justSent = true;
    tft_resetBar();

  } else {

    /****************************************
     *               RX Cycle               *
     ****************************************/
    while (!timeWait());

    /*---------------------------------------------------------------*
     *  Collect energy information for 12.8 secs and pre-process     *
     *  magnitudes found                                             *
     *---------------------------------------------------------------*/
    _INFOLIST("%s ------- RX w[%d]----------\n", __func__,getQSOwindow());

    inc_collect_power();
    /*---------------------------------------------------------------*
     *  Evaluate magnitudes captured and decode messages on passband *
     *---------------------------------------------------------------*/
    uint32_t decode_begin = time_us_32();
    num_decoded = decode_ft8(message_list);

    /*---------------------------------------------------------------*
     *  Transform decode symbols into actual ft8 messages            *
     *---------------------------------------------------------------*/
    tdecode = time_us_32() - decode_begin;
    identify_message_types(message_list, my_callsign);

    /*---------------------------------------------------------------*
     * Walk the message, place them on the QSO text area of the GUI  * 
     * Color mark the different messages according to it's nature    *
     * CQ  (Green)                                                   *
     * QSO (Red)                                                     *
     * Other QSO (white)                                             *
     * My CQ (Yellow)                                                *
     *---------------------------------------------------------------*/
    for (int i = 0; i < num_decoded; i++) {
      _INFOLIST("%s [%02d] w[%d] %04d %4d %s\n", __func__, i, message_list[i].qsowindow,message_list[i].af_frequency, message_list[i].self_rx_snr, message_list[i].full_text);
      sprintf(msg,"%04d %4d %s", message_list[i].af_frequency, message_list[i].self_rx_snr, message_list[i].full_text);
      uint16_t qsotype=0;
      if (message_list[i].addressed_to_me) {
         qsotype=3; 
      } else {
         if (message_list[i].type_cq) {
            qsotype=1;
         }
      }
      tft_storeQSO(message_list[i].qsowindow,qsotype,msg,message_list[i].af_frequency,message_list[i].self_rx_snr,message_list[i].station_callsign,message_list[i].grid_square);

    }
    /*------------------------------------------------------*
     * Mark the receiving cycle as completed                *
     */
    justSent = false;
    tft_resetBar();
  }

  /*******************************************************
   *         FT8 Evaluation Finite State Machine         *
   *******************************************************/
  /*------------------------------------------------------------
   * Validate the conditions for the FSM to evaluate messages
   */  
  if ((state != 0 && !justSent) || (autosend && !justSent) || (triggerCQ && !justSent) || (triggerCALL && !justSent)) {   //*Fix BUILD 41
    send = ft8bot(&CurrentStation, &sendChoices, message_list);
  } else {
    send = false;
  }

  /*--------------------
     reset the message list to start a new cycle
  */
  memset(message_list, 0, sizeof(message_list));
  return;
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                             setup() (core0)                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup()
{

#ifdef DEBUG
  _SERIAL.begin(115200);
  while (!_SERIAL);
  delay(200);
  _SERIAL.flush();
#endif //DEBUG

  /*-----------
     Data area initialization
  */
  strcpy(my_callsign, MY_CALLSIGN);
  strcpy(my_grid, MY_GRID);
  strcpy(programname,PROGNAME);
  strcpy(version,VERSION);
  strcpy(build,BUILD);
  strcpy(ip,"127.0.0.1");
  
  /*-----------
     System initialization
  */
  INIT();
  initSi5351();
  tft_setup();

  /*--------
     If DOWN is found pressed on startup then enters calibration mode
  */

  if ( digitalRead(DOWN) == LOW ) {

    _INFOLIST("%s Auto calibration mode started\n", __func__);
    tft_autocal();
    AutoCalibration();
  }

  /*-------[RP2040]------ Manual time-sync feature
     if UP is found pressed on startup then enters manual time sync mode

  */

  if ( digitalRead(UP) == LOW ) {

    _INFOLIST("%s Manual time-sync mode\n", __func__);
    timeSync();

  }
  /*--- end of time-sync feature ----*/


  /*--------------------
     Place the receiver in reception mode
  */
  digitalWrite(RX, LOW);
  
  /*--------------------
     Setup and initialize the FT8 structures
  */

  setup_ft8();
  delay(1000);
  tft_init();
  tft_updateBand();
  _INFOLIST("%s *** Transceiver ready ***\n", __func__);

}
//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************

void loop()
{


  /*-----------------------------------------------------------------------------*
                            Periodic dispatcher section
     Housekeeping functions that needs to be run periodically (for not too long)
    -----------------------------------------------------------------------------*/

  /*------------------------------------------------
     Explore and handle interactions with the user
     thru the UP/DOWN or TX buttons
  */
  checkButton();
  tft_checktouch();
  tft_run();


  /*------------------------------------------------
     Main FT8 handling cycle
  */
  ft8_run();

}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                         end of loop()                                                      *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                       Support functions                                                    *
//                       (mostly original from ADX_UnO except few debug messages)                             *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void initSi5351() {

  //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  //----------------------------------------------------------------------------------
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for reduced power for RX

  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn on calibration Clock
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Turn off transmitter clock

  si5351.output_enable(SI5351_CLK0, 0);   //RX off
  si5351.output_enable(SI5351_CLK2, 0);   //RX off

  si5351.set_freq(freq * 100ULL, SI5351_CLK1);

  si5351.set_clock_pwr(SI5351_CLK1, 1); // Turn on receiver clock
  si5351.output_enable(SI5351_CLK1, 1);   // RX on

}
/*----------------------------------------------------
   check buttons and operate with them as appropriate
*/
void checkButton() {

  UP_State = digitalRead(UP);
  DOWN_State = digitalRead(DOWN);

  /*----
     UP(Pressed) && DOWN(Pressed) && !Transmitting
     Start band selection mode
  */

  if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
    delay(100);
    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);
    if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
       Band_Select();
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
      ManualTX();
    }
  }

}
/*-----------
   Manual timesync procedure
   The operation is held whilst the UP button is kept pressed, the user needs to wait to the top of the
   minute (sec=00) to release it, upon release the second offset is computed and used to align the time
   with that synchronization.
*/
void timeSync() {

  bool flipLED = true;
  uint32_t ts = millis();
  now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeprev);
  _INFOLIST("%s Initial time=[%02d:%02d:%02d]\n", __func__, timeprev.tm_hour, timeprev.tm_min, timeprev.tm_sec);
  while (digitalRead(UP) == LOW) {
    delay(500);
    if (millis() - ts > 500) {
      digitalWrite(WSPR, flipLED);
      digitalWrite(JS8, flipLED);
      digitalWrite(FT4, flipLED);
      digitalWrite(FT8, flipLED);
      flipLED = !flipLED;
      ts = millis();
    }
  }
  t_ofs = time(nullptr);
  digitalWrite(WSPR, false);
  digitalWrite(JS8, false);
  digitalWrite(FT4, false);
  digitalWrite(FT8, false);
  now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeprev);
  _INFOLIST("%s Manual time sync=[%02d:%02d:%02d]\n", __func__, timeprev.tm_hour, timeprev.tm_min, timeprev.tm_sec);

}

/*--------------------------------------------
 * Changes ADX hardware to start TX
 */
void startTX() {

  unsigned long freq1 = freq;
  digitalWrite(RX, LOW);
  si5351.set_clock_pwr(SI5351_CLK0, 1); // Turn on receiver clock
  si5351.set_clock_pwr(SI5351_CLK1, 0); // Turn on receiver clock
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn on receiver clock

  si5351.output_enable(SI5351_CLK1, 0);   //RX off
  si5351.output_enable(SI5351_CLK2, 0);   //RX off
  digitalWrite(TX, 1);
  si5351.set_freq(freq1 * 100ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   //TX on
  TX_State = 1;
  tft_set(BUTTON_TX,1);

  _INFOLIST("%s TX+ f=%lu freqx=%lu \n", __func__, freq, freq1);

}
/*------------------------------------
 * Changes ADX hardware to stop TX
 */
void stopTX() {

  digitalWrite(TX, 0);
  si5351.set_freq(freq * 100ULL, SI5351_CLK0);

  si5351.set_clock_pwr(SI5351_CLK0, 0); // Turn on receiver clock
  si5351.set_clock_pwr(SI5351_CLK1, 1); // Turn on receiver clock
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn on receiver clock

  si5351.output_enable(SI5351_CLK0, 0);   //TX off
  si5351.output_enable(SI5351_CLK1, 1);   //TX off
  si5351.output_enable(SI5351_CLK2, 0);   //TX off

  TX_State = 0;
  _INFOLIST("%s TX-\n", __func__);
  tft_set(BUTTON_TX,0);
  
}
/*-------------------------------------
 * Manual TX function (push TX button)
 */
void ManualTX() {

  startTX();
  while (digitalRead(TXSW)==LOW) ;
  stopTX();

}

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
}
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

  updateEEPROM();
  Band_assign();
  freq=Slot2Freq(Band_slot);
  tft_updateBand();

  _INFOLIST("%s completed Band assignment Band_slot=%d freq=%lu\n", __func__, Band_slot,freq);


}
void updateEEPROM() {

  addr = 50;
  EEPROM.put(addr, Band_slot);
  EEPROM.commit();

}
//*********************************[ END OF BAND SELECT ]*****************************



//************************** [SI5351 VFO Calibration Function] ************************
void Calibration() {

  unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
  si5351.output_enable(SI5351_CLK0, 0);   // RX on
  si5351.output_enable(SI5351_CLK1, 0);   // RX on

  si5351.set_clock_pwr(SI5351_CLK0, 0); // Turn on receiver clock
  si5351.set_clock_pwr(SI5351_CLK1, 0); // Turn on receiver clock

  si5351.set_clock_pwr(SI5351_CLK2, 1); // Turn on receiver clock
  si5351.output_enable(SI5351_CLK2, 1);   // RX on


  unsigned long F_FT8;
  unsigned long F_FT4;
  unsigned long F_JS8;
  unsigned long F_WSPR;

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
      si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK2);




      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
      si5351.output_enable(SI5351_CLK2, 1);   // RX on


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
      EEPROM.commit();


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
/*-----------------------------------------------
 * Convert a slot number to the FT8 frequency for
 * that slot
 */
unsigned long Slot2Freq(int s) {
  if (s<1 || s>BANDS) {
      s=1;
  }
  uint16_t b=Bands[s-1];
  uint8_t  i=0;
  switch (b) {
    case 80 : i=0;break;
    case 60 : i=1;break;
    case 40 : i=2;break;
    case 30 : i=3;break;
    case 20 : i=4;break;
    case 17 : i=5;break;
    case 15 : i=6;break;
    case 12 : i=7;break;
    case 10 : i=8;break;
  }
  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);
  digitalWrite(8-Band_slot,HIGH);

  return slot[i];

}
/*----------------------------------------------
 * ADX board initializacion
 * I/O definition
 * I2C initialization
 * Wire setup
 * EEPROM parameters
 */
void INIT() {

  /*-----------------------------
     Port definitions (pinout, direction and pullups used
  */
  
  /*--------
     Initialize switches
  */
  gpio_init(UP);
  gpio_init(DOWN);
  gpio_init(TXSW);

  /*----
     Initialize RX command
  */
  gpio_init(RX);

  /*---
     Initialize LED
  */

  gpio_init(WSPR);
  gpio_init(JS8);
  gpio_init(FT4);
  gpio_init(FT8);
  gpio_init(TX);

  /*-----
     Set direction of input ports
  */
  gpio_set_dir(UP, GPIO_IN);
  gpio_set_dir(DOWN, GPIO_IN);
  gpio_set_dir(TXSW, GPIO_IN);

  /*-----
     Pull-up for input ports
     (Warning! See hardware instructions
  */

  gpio_pull_up(TXSW);
  gpio_pull_up(DOWN);
  gpio_pull_up(UP);

  /*---
     Set output ports
  */
  gpio_set_dir(RX, GPIO_OUT);
  gpio_set_dir(TX, GPIO_OUT);

  gpio_set_dir(WSPR, GPIO_OUT);
  gpio_set_dir(JS8, GPIO_OUT);
  gpio_set_dir(FT4, GPIO_OUT);
  gpio_set_dir(FT8, GPIO_OUT);

  /*----
     Digital input pin, it's an ADC port to allow further development of DSP based inputs
  */

  gpio_init(FSKpin);
  gpio_set_dir(FSKpin, GPIO_IN);
  gpio_pull_up(FSKpin);

  /*---
     Initialice I2C sub-system
  */

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();

  _INFOLIST("%s: I/O setup completed\n", __func__);


  /*------
     initialize working parameters if stored in EEPROM
  */

  EEPROM.begin(512);
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
    EEPROM.commit();


  }

  else

  {
    //--------------- EEPROM INIT VALUES
    addr = 30;
    EEPROM.get(addr, temp);

    addr = 10;
    EEPROM.get(addr, cal_factor);

    addr = 50;
    EEPROM.get(addr, Band_slot);



  }
  freq=Slot2Freq(Band_slot);
  tft_updateBand();

}
//********************************[ END OF INITIALIZATION FUNCTION ]*************************************
/*----------------------
 * Interrupt handler to perform a frequency counting used in calibration
 */
void pwm_int() {
  pwm_clear_irq(pwm_slice);
  f_hi++;
}

void setCalibrationLED(uint16_t e) {

  if (e>75) {
     digitalWrite(WSPR,HIGH);
     digitalWrite(JS8,HIGH);
     digitalWrite(FT4,HIGH);
     digitalWrite(FT8,HIGH);
     return;
  }
  if (e>50) {
     digitalWrite(WSPR,HIGH);
     digitalWrite(JS8,HIGH);
     digitalWrite(FT4,HIGH);
     digitalWrite(FT8,LOW);
     return;
  }
  if (e>25) {
     digitalWrite(WSPR,HIGH);
     digitalWrite(JS8,HIGH);
     digitalWrite(FT4,LOW);
     digitalWrite(FT8,LOW);
     return;
  }

  if (e>10) {
     digitalWrite(WSPR,HIGH);
     digitalWrite(JS8,LOW);
     digitalWrite(FT4,LOW);
     digitalWrite(FT8,LOW);
     return;
  }
  digitalWrite(WSPR,LOW);
  digitalWrite(JS8,LOW);
  digitalWrite(FT4,LOW);
  digitalWrite(FT8,LOW);
  return;

}
//***************************[SI5351 VFO Auto-Calibration Function]********************
//* This function has no equivalent on the ADX-UnO firmware and can only be activated
//* with the RDX or ADX2RDX boards
//*
//* To enable uncomment the #define AUTOCAL     1 statement
//*************************************************************************************
void AutoCalibration () {

bool b = false;

  if (!Serial) {
     Serial.begin(115200);
     Serial.flush();
  }
  sprintf(hi,"Autocalibration procedure started\n");
  Serial.print(hi);
  tft_print(hi);

  //tft_print(50,70,hi);

  sprintf(hi,"cal_factor=%d\n",cal_factor);
  Serial.print(hi);
  tft_print(hi);
  

  addr = 10;
  EEPROM.get(addr, cal_factor);

  sprintf(hi,"cal_factor reset\n");
  Serial.print(hi);
  tft_print(hi);
 
  cal_factor=0;
  EEPROM.put(addr, cal_factor);
  EEPROM.commit();

  digitalWrite(TX,LOW);
  setCalibrationLED(1000);
  
  while (!digitalRead(DOWN));
  /*--------------------

  */
  gpio_init(CAL);
  gpio_pull_up(CAL);
  gpio_set_dir(CAL, GPIO_IN);
  delay(10);

  /*----
    Prepare Si5351 CLK2 for calibration process
    ---*/

  gpio_set_function(CAL, GPIO_FUNC_PWM); // GP9
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Enable the clock for calibration
  si5351.set_clock_pwr(SI5351_CLK1, 0); // Enable the clock for calibration
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(Cal_freq * 100UL, SI5351_CLK2);

  sprintf(hi,"Si5351 f=%lu MHz\n",(unsigned long)Cal_freq);
  Serial.print(hi);
  tft_print(hi);
  
  /*--------------------------------------------*
     PWM counter used for automatic calibration
     -------------------------------------------*/
  fclk = 0;
  int16_t n = int16_t(CAL_COMMIT);
  cal_factor = 0;

  pwm_slice = pwm_gpio_to_slice_num(CAL);

  /*---------------------------------------------*
    Perform a loop until convergence is achieved
  */
  while (true) {
    /*-------------------------*
       setup PWM counter
      -------------------------*/
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_init(pwm_slice, &cfg, false);
    gpio_set_function(CAL, GPIO_FUNC_PWM);

    pwm_set_irq_enabled(pwm_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    f_hi = 0;

    /*---------------------------*
       PWM counted during 1 sec
      ---------------------------*/
    uint32_t t = time_us_32() + 2;
    while (t > time_us_32());
    pwm_set_enabled(pwm_slice, true);
    t += 1000000;
    while (t > time_us_32());
    pwm_set_enabled(pwm_slice, false);

    /*----------------------------*
       recover frequency in Hz
      ----------------------------*/
    fclk = pwm_get_counter(pwm_slice);
    fclk += f_hi << 16;
    error = fclk - Cal_freq;
    sprintf(hi,"n(%01d) cal(%lu) Hz dds(%lu) Hz err (%lu) Hz factor(%lu)\n",n, (unsigned long)Cal_freq, (unsigned long)fclk, (unsigned long)error, (unsigned long)cal_factor);
    Serial.print(hi);
    sprintf(hi,"cal(%lu) e(%lu) Hz cf(%lu)\n",(unsigned long)Cal_freq,(unsigned long)error,(unsigned long)cal_factor);
    tft_print(hi);
    setCalibrationLED((uint16_t)error);
    tft_error((uint16_t)error);

    if (labs(error) > int32_t(CAL_ERROR)) {
        b = !b;
        if (b) {  
          digitalWrite(TX, LOW);
        } else {
          digitalWrite(TX, HIGH);
        }
        if (error < 0) {
           cal_factor = cal_factor - CAL_STEP;
        } else {
           cal_factor = cal_factor + CAL_STEP;
        }
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    } else {
      n--;
      if (n == 0) {
        break;
      }

    }
  }  //larger while(true) loop
  
  EEPROM.put(addr, cal_factor);
  EEPROM.commit();
  setCalibrationLED(0);
  
  sprintf(hi,"Completed cal_factor=%d\n",cal_factor);
  Serial.print(hi);
  tft_print(hi);

  sprintf(hi,"Power-off and re-start\n");
  Serial.print(hi);
  tft_print(hi);
  
  while (true) {
    
  }

}
