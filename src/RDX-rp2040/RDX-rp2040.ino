//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              RDX_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2022
//
// Version 1.0
//
// This is the implementation of a rp2040 firmware for a monoband, self-contained FT8 transceiver based on
// the ADX hardware architecture.
//
//*********************************************************************************************************
// Required Libraries and build chain components
//
// Created with Arduino IDE using the Arduino-Pico core created by Earle F. Philhower, III available
// at https://github.com/earlephilhower
// Check for installation and configuration instructions at https://www.upesy.com/blogs/tutorials/install-raspberry-pi-pico-on-arduino-ide-software
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
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*************************************[ LICENCE and CREDITS ]*********************************************
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
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
//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
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

/*-------------------------------------------------
   IDENTIFICATION DIVISION.
   (just a programmer joke)
*/
#define PROGNAME "RDX_rp2040"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION 1.0
#define BUILD     1

/*------------------------------------------------------
   Main variables
*/
char hi[128];
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

/*------------------------------------------------------
   Set internal time by default
*/
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
bool stopCore1 = true;

/*-------------------------------------------------------
 * ft8 definitions
 */
 
message_info CurrentStation;
UserSendSelection sendChoices;
message_info message_list[kMax_decoded_messages]; // probably needs to be memset cleared before each decode


bool send = false;
bool justSent = false; //must recieve right after sending
bool autosend = true;
bool cq_only = false;
uint64_t config_us;
uint64_t fine_offset_us = 0; //in us
int16_t signal_for_processing[num_samples_processed] = {0};
uint32_t handler_max_time = 0;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                             Global Variables                                             *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
uint32_t val;
int temp;
uint32_t val_EE;
int addr = 0;
int Band_slot = 0;
int Band = 0;

int Band1 = 40; // Band 1 // These are default bands. Feel free to swap with yours
int Band2 = 30; // Band 2
int Band3 = 20; // Band 3
int Band4 = 10; // Band 4 //*RP2040* changed to 10m from 17m

unsigned long freq = 7074000UL;

int32_t cal_factor;
int TX_State = 0;

int UP_State;
int DOWN_State;
int TXSW_State;
int Bdly = 250;

Si5351 si5351;
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  core1 processing                                                                        *
//*  irq handler to process samples in the background using core1                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void core1_irq_handler()
{

/*-------------------------------------
 * 
 * Copy capture_buf to signal_for_processing array, take fft and save to power, check for user keypad input
 */
    while (rp2040.fifo.available())
    
    {   
        uint32_t handler_start = time_us_32();
        uint32_t fdx = rp2040.fifo.pop();
        uint16_t idx = (uint16_t) fdx;

/*-------
 * extract signal for processing
 */
        for(int i = 0; i < nfft; i++){
            fresh_signal[i] -= DC_BIAS;
        }
        inc_extract_power(fresh_signal);
/*-------
 * processing shouldn't last more than 160 mSecs
 */
        uint32_t handler_time = (time_us_32() - handler_start)/1000;
        if (handler_time > handler_max_time){
            handler_max_time = handler_time;
        }
    }
    multicore_fifo_clear_irq(); // Clear interrupt
}

/*-------------------------------
 * setup1()
 * core1 is held from processing till released from the main setup() when ready to go
 */
void setup1(void)

{
/*--------------------  
 * wait till cleared
 */
    uint32_t tc1 = time_us_32();
    while (stopCore1) {
      while (time_us_32()-tc1 < 1000) {}
      tc1=time_us_32();
    }

 /*---------   
  * configure core1 interrupt handler
  */
    printf("second core running!\n");
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_irq_handler);
    irq_set_enabled(SIO_IRQ_PROC1, true);

/*----------
 * wait forever, actual processing is made at the interrupt handler
 * the delay is just to avoid a smart-ass compiler optimization to 
 * remove the infinite loop altogether
 */
    while (true)
    {
        delay(1000);
    }
}
/*-----------------
 * setup ft8 operation
 */
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  ft8 processing                                                                          *
//*  setup and handle the ft8 processing, this is actually the orchestation of the ft8lib    *
//*  functions by Karliss Goba                                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
 void setup_ft8() {

/*--------
 * the original overclock the processor, the initial version of this
 * firmware won't
 */
    //overclocking the processor
    //133MHz default, 250MHz is safe at 1.1V and for flash
    //if using clock > 290MHz, increase voltage and add flash divider
    //see https://raspberrypi.github.io/pico-sdk-doxygen/vreg_8h.html

    //vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
    //@@@ set_sys_clock_khz(250000,true);

/*------
 * setup the ADC processing
 */
    setup_adc();

/*------
 * make the Hanning window for fft work
 */
    make_window();
    

/*------    
 * freed the semaphore which where core1 is being held, so start it
 */
    stopCore1=false;

/*------
 * wait to settle
 */

    sleep_ms(1000);   
    
    
} //end of the ft8 setup

void ft8_run() {
    
        char message[25] = {0};
        uint8_t tones[79] = {0};

        if (send)
        {
            if (autosend)
            {
                printf("autosending\n");
                printf("still here? %s\n", CurrentStation.station_callsign);
                auto_gen_message(message, CurrentStation, MY_CALLSIGN, MY_GRID);
            }
            else
            {
                manual_gen_message(message, CurrentStation, sendChoices, MY_CALLSIGN, MY_GRID);
                //reset send choices
                sendChoices.call_cq = sendChoices.send_73 = sendChoices.send_grid = sendChoices.send_RR73 = sendChoices.send_RRR = sendChoices.send_Rsnr = sendChoices.send_snr = false;
            }

            printf("message to be sent: %s\n", message);
            generate_ft8(message, tones);
            //display message in my messages in yellow
            //add_to_my_messages(message, 0xFFE0);

            printf("SENDING FOR 12.64 SECONDS\n\n");
            //gpio_put(LED_PIN, 1);
            // ---> send_ft8(tones, rf_freq, CurrentStation.af_frequency);
            /*-----[ADX-rp2004-plus]--------
             * integration fix with ADX, freq held the base frequency
             */
            send_ft8(tones, freq, CurrentStation.af_frequency);
            //gpio_put(LED_PIN, 0);
            send = false;
            justSent = true;
        }

        else
        {
            printf("RECIEVING FOR 12.8 SECONDS\n\n");

            inc_collect_power();

            printf("Handler max time: %d\n", handler_max_time);
        
            uint32_t decode_begin = time_us_32();
            uint8_t num_decoded = decode_ft8(message_list);
            printf("decoding took this long: %d us\n", time_us_32() - decode_begin);

            decode_begin = time_us_32(); //i'll just reuse this variable

            //testing example
/*             message_list[0].self_rx_snr = -23;
            message_list[0].time_offset = -0.1;
            message_list[0].af_frequency = 2222;
            strcat(message_list[0].full_text, "AA1GD KI5KGU R+04"); */
            
            //message_list[0].addressed_to_me = true;

            identify_message_types(message_list, MY_CALLSIGN);
            printf("identification took this long: %d us\n", time_us_32() - decode_begin);
            printf("finished identifying message types\n");
            justSent = false;

            //reset_progress_bar(); 
            //update_current_time(&current_hms, &config_hms, config_us);
            //update_time_display(&current_hms);

           // update_main_messages(message_list, num_decoded, cq_only, &current_hms);

            //update_main_messages(message_list,1,false,&current_hms);
        }

        int selected_station = -1; //default is no response
        int rTB = -1;              //rTB stands for responseTypeBuffer

        printf("\nWAITING 2 SECONDS FOR USER INPUT\n");
        printf("Enter station to respond to: ");

        if (autosend){
            for (int i = 0; i < kMax_decoded_messages; i++){
                if (message_list[i].addressed_to_me){
                    selected_station = i; //if autosending, finds the first occurence of callsign and automatically responds
                    printf("autoselected to %d\n", selected_station);
                    send = true;
                    break;
                }
            }
        }
        else
        {
            printf("Enter response type: 0 for CQ, 1 for grid, 2 for snr, 3 for Rsnr, 4 for RRR, 5 for RR73, 6 for 73, 7+ for no choice:\n");
        }

        //while modulo greater than 12.8, OR aborted
        
        //@@@@ uint64_t overtime = (time_us_64() - config_us + config_hms.sec * 1000000) % 15000000;
        //@@@@ here it needs to be sync with 15 secs edge
        //This is to react to input       
 /*       
        uint64_t overtime=0;
        if (overtime < 12800000){
            printf("no time to select\n");
            sleep_us(13000000 - overtime);
        }
        while ((time_us_64() - config_us + config_hms.sec * 1000000) % 15000000 > 12800000)
        {
            //scan for keypad input... otherwise do nothing
            
            
            char char_selection = pico_keypad_get_key();

            if (char_selection == '*')
            { //make this the abort/menu key
                //goto back to setup
                goto reconfigure;
            }
            
            
            if (justSent)
            {
                continue;
            }

            if (!send || autosend)
            {
                selected_station = char_to_int(char_selection);
            }
            else if (!autosend)
            {
                rTB = char_to_int(char_selection);
            }

            if (selected_station < 0)
            {
                send = false;
            }
            else if (selected_station < kMax_decoded_messages)
            {
                send = true;
            }

        } //end of while loop
*/
        
/*
        if (!autosend)
        {
            if (rTB == 0)
            {
                sendChoices.call_cq = true;
            }
            else if (rTB == 1)
            {
                sendChoices.send_grid = true;
            }
            else if (rTB == 2)
            {
                sendChoices.send_snr = true;
            }
            else if (rTB == 3)
            {
                sendChoices.send_Rsnr = true;
            }
            else if (rTB == 4)
            {
                sendChoices.send_RRR = true;
            }
            else if (rTB == 5)
            {
                sendChoices.send_RR73 = true;
            }
            else if (rTB == 6)
            {
                sendChoices.send_73 = true;
            }
        }
*/
/*       
        //need to make autosend automatically select the station
        if (send)
        {
            bool same_station = strcmp(CurrentStation.station_callsign,message_list[selected_station].station_callsign);
            CurrentStation = message_list[selected_station];
            //update_current_station_display(CurrentStation, same_station);
            printf("CurrentStation set to %d\n",selected_station);
            printf("can callsign be seen? %s\n", CurrentStation.station_callsign);
        }
*/
        //reset message_list
        printf("clearing %d bytes of message_list\n", sizeof(message_list));
        memset(message_list, 0, sizeof(message_list));
        printf("done a loop!\n");
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
     System initialization
  */
  INIT();
  initSi5351();

  /*--------
     If DOWN is found pressed on startup then enters calibration mode
  */

  if ( digitalRead(DOWN) == LOW ) {

    _INFOLIST("%s Calibration mode started\n", __func__);
    Calibration();
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

  _INFOLIST("%s finalized Ok\n", __func__);

}

//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{


  /*-----------------------------------------------------------------------------*
                            Periodic dispatcher section
     Housekeeping functions that needs to be run periodically (for not too long)
    -----------------------------------------------------------------------------*/


  /*---------------------------------------------------------*
     Periodic time synchronization test
  */
  time_t now = time(0) - t_ofs;
  gmtime_r(&now, &timeinfo);
  if (timeinfo.tm_min != timeprev.tm_min) {
    _INFOLIST("%s time=[%02d:%02d:%02d]\n", __func__, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    timeprev = timeinfo;
  }
  /*------------------------------------------------
     Explore and handle interactions with the user
     thru the UP/DOWN or TX buttons
  */
  checkButton();

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
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Turn off Calibration Clock
  si5351.set_freq(freq * 100ULL, SI5351_CLK1);
  si5351.set_clock_pwr(SI5351_CLK1, 1); // Turn off Calibration Clock

  _INFOLIST("%s si5351 clock initialization completed\n", __func__);

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
  time_t now = time(nullptr) - t_ofs;
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
  _INFOLIST("%s Manual time-synced\n", __func__);
  now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeprev);
  _INFOLIST("%s Sync time=[%02d:%02d:%02d]\n", __func__, timeprev.tm_hour, timeprev.tm_min, timeprev.tm_sec);

}


//*******************************[ Manual TX FUNCTION ]********************************
void ManualTX() {

  unsigned long freq1 = freq;
  digitalWrite(RX, LOW);
  si5351.output_enable(SI5351_CLK1, 0);   //RX off
  _INFOLIST("%s TX+\n", __func__);


TXON:

  TXSW_State = digitalRead(TXSW);
  digitalWrite(TX, 1);
  si5351.set_freq(freq1 * 100ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   //TX on
  TX_State = 1;

  if (TXSW_State == HIGH) {
    goto EXIT_TX;

  }
  goto TXON;

EXIT_TX:
  digitalWrite(TX, 0);
  si5351.output_enable(SI5351_CLK0, 0);   //TX off
  TX_State = 0;
  _INFOLIST("%s TX-\n", __func__);

}

//********************************[ END OF Manual TX ]*********************************
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

  _INFOLIST("%s band_slot=%d freq=%ld\n", __func__, Band_slot, freq);
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
      _INFOLIST("%s <UP> Band_slot=%d\n", __func__, Band_slot);

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
      _INFOLIST("%s <DOWN> Band_slot=%d\n", __func__, Band_slot);

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

  addr = 50;
  EEPROM.put(addr, Band_slot);
  EEPROM.commit();

  Band_assign();
  _INFOLIST("%s completed set Band_slot=%d\n", __func__, Band_slot);


}

//*********************************[ END OF BAND SELECT ]*****************************



//************************** [SI5351 VFO Calibration Function] ************************

void Calibration() {

  unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz

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
      si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration


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

  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

  _INFOLIST("%s completed freq=%ld\n", __func__, freq);

}
//********************************[ END OF INITIALIZATION FUNCTION ]*************************************
