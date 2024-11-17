//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              dds_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2024
//
// Version 1.0
//
// This is a DDS implementation for evaluation purposes using the library contained in the package
// The program produces a single frecuency RF signal
//*-------------------------------------------------------------------------------------------------------------
// https://github.com/RPiks/pico-hf-oscillator 
// by
// Roman Piksaykin [piksaykin@gmail.com], R2BDY
// https://www.qrz.com/db/r2bdy
//
//  Porting by Dr. Pedro E. Colla (LU7DZ). 2024
//  V1.0 Initial porting effort
//*-------------------------------------------------------------------------------------------------------------
// This firmware is meant to be compiled using the latest Arduino IDE environment with the following parameters
//
// Board: "Raspberry Pi Pico"
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
//*-------------------------------------------------------------------------------------------------------------
//*                                                License
//*-------------------------------------------------------------------------------------------------------------
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
//*-------------------------------------------------------------------------------------------------------------
//
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://www.qrz.com/db/r2bdy
//
//  DESCRIPTION
//
//      The oscillator provides precise generation of any frequency ranging
//  from 1 Hz to 33.333 MHz with tenth's of millihertz resolution (please note that
//  this is relative resolution owing to the fact that the absolute accuracy of 
//  onboard crystal of pi pico is limited; the absoulte accuracy can be provided
//  when using GPS reference option included).
//      The DCO uses phase locked loop principle programmed in C and PIO asm.
//      The DCO does *NOT* use any floating point operations - all time-critical
//  instructions run in 1 CPU cycle.
//      Currently the upper freq. limit is about 33.333 MHz and it is achieved only
//  using pi pico overclocking to 270 MHz.
//      Owing to the meager frequency step, it is possible to use 3, 5, or 7th
//  harmonics of generated frequency. Such solution completely cover all HF and
//  a portion of VHF band up to about 233 MHz.
//      Unfortunately due to pure digital freq.synthesis principle the jitter may
//  be a problem on higher frequencies. You should assess the quality of generated
//  signal if you want to emit a noticeable power.
//      This is an experimental project of amateur radio class and it is devised
//  by me on the free will base in order to experiment with QRP narrowband
//  digital modes.
//      I appreciate any thoughts or comments on this matter.
//
//  REVISION HISTORY
// 
//      Rev 1.0   16 Nov 2024   Initial creation
//
//  PROJECT PAGE
//      https://github.com/lu7did/ADX-rp2040/tree/master
//
//  LICENCE
//      MIT License (http://www.opensource.org/licenses/mit-license.php)
//
//  Copyright (c) 2023 by Roman Piksaykin
//  Copyright (c) 2024 by Dr. Pedro E. Colla (LU7DZ)
//*-------------------------------------------------------------------------------------------------------------
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
//*-------------------------------------------------------------------------------------------------------------
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "defines.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"

#include "hwdefs.h"
#include "dds_rp2040.h"
#include "piodco.h"
#include "dco2.pio.h"
#include "protos.h"

#include <NeoPixelConnect.h>

/*------------------------------------------------------
 *   Internal clock handling
 */
struct tm timeinfo;        //current time
struct tm timeprev;        //epoch time
time_t t_ofs = 0;          //time correction after sync (0 if not sync-ed)
time_t now;
char timestr[12];
struct semaphore spc;
float timezone=TIMEZONE;
int   tzh=0;
int   tzm=0;
int   localHour=0;
int   localMin=0;
char hi[80];

uint32_t core1_stack[STACK_SIZE];
PioDco DCO; /* External in order to access in both cores. */
//NeoPixelConnect led(pin_led, 1, pio0, 0);  // creamos la instacia para utilizar el led, con el nombre de led


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                 Dedicated core DDS function
//* Running on PIO processor
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void core1_entry()
{
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;

    /* Initialize DCO */
    PioDCOInit(&DCO, 6, clkhz);
    /* Run DCO. */
    PioDCOStart(&DCO);
    /* Set initial freq. */
    PioDCOSetFreq(&DCO, GEN_FRQ_HZ, 0u);
    /* Run the main DCO algorithm. It spins forever. */
    PioDCOWorker2(&DCO);
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                             Initial Setup
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup() {

  _SERIAL.begin(115200);
  _SERIAL.setTimeout(4);

  _INFO("Program %s version %s(%s)\n",PROGNAME,VERSION,BUILD);


  /*-----------------------------------------
   * define special semaphore to control 
   * access to the serial port while 
   * debugging avoiding recursion and 
   * re-entrancy problems.
   */
  sem_init(&spc, 1, 1);

//*-------------------------------------------------------------------------------------------*
//*from AA1GD*
//overclocking the processor
//133MHz default, 250MHz is safe at 1.1V and for flash
//if using clock > 290MHz, increase voltage and add flash divider
//see https://raspberrypi.github.io/pico-sdk-doxygen/vreg_8h.html

  const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;
  set_sys_clock_khz(clkhz / 1000L, true);
  _INFO("System clock set to %ld KHz",clkhz/1000L);

  /*--------
     Initialize switches
  */
  gpio_init(UP);
  gpio_init(DOWN);
  gpio_init(TXSW);
  gpio_init(PICO_DEFAULT_LED_PIN);

  /*-----
     Set direction of input ports
  */
  gpio_set_dir(UP, GPIO_IN);
  gpio_set_dir(DOWN, GPIO_IN);
  gpio_set_dir(TXSW, GPIO_IN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  /*-----
     Pull-up for input ports
     (Warning! See hardware instructions
  */

  gpio_pull_up(TXSW);
  gpio_pull_up(DOWN);
  gpio_pull_up(UP);




  multicore_launch_core1_with_stack (core1_entry,core1_stack,STACK_SIZE);
  _INFO("%s: System initialization completed\n", __func__);

/*
  This is to change frequency
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ - 5*(rndval & 7), 0u);
*/
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                             Loop
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void loop() {

/*
    led.neoPixelFill(0, 0, 255, true); // Encendemos el led , en este ejemplo encendemos el AZUL
    delay(1000);
    led.neoPixelClear(true); // Apagamos el led
    delay(1000);
*/
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(5);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(5);
}
