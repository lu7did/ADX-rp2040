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
///////////////////////////////////////////////////////////////////////////////
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
//      https://github.com/RPiks/pico-hf-oscillator
//
//  LICENCE
//      MIT License (http://www.opensource.org/licenses/mit-license.php)
//
//  Copyright (c) 2023 by Roman Piksaykin
//  
//  Permission is hereby granted, free of charge,to any person obtaining a copy
//  of this software and associated documentation files (the Software), to deal
//  in the Software without restriction,including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "defines.h"

#include "piodco.h"
//#include "build/dco2.pio.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdio/driver.h"

#include "./lib/assert.h"
#include "./debug/logutils.h"
#include "hwdefs.h"

#include "GPStime.h"

#include "hfconsole.h"

#include "protos.h"

//#define GEN_FRQ_HZ 32333333L
#define GEN_FRQ_HZ 29977777L

PioDco DCO; /* External in order to access in both cores. */

int main() 
{
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;
    set_sys_clock_khz(clkhz / 1000L, true);

    //*----> Revisar con que reemplazar stdio_init_all();
    sleep_ms(1000);
    printf("Start\n");

    HFconsoleContext *phfc = HFconsoleInit(-1, 0);
    
    HFconsoleSetWrapper(phfc, (void *) ConsoleCommandsWrapper);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

    for(;;)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(5);
        int r = HFconsoleProcess(phfc, 10);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1);
    }

    for(;;)
    {
        sleep_ms(100);
        int chr = getchar_timeout_us(100);//getchar();
        printf("%d %c\n", chr, (char)chr);
    }
  
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

    //SpinnerDummyTest();
    //SpinnerSweepTest();
    //SpinnerMFSKTest();
    SpinnerRTTYTest();
    //SpinnerMilliHertzTest();
    //SpinnerWide4FSKTest();
    //SpinnerGPSreferenceTest();
}

/* This is the code of dedicated core. 
   We deal with extremely precise real-time task. */
void core1_entry()
{
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;

    /* Initialize DCO */
    //*FIX* assert_(0 == PioDCOInit(&DCO, 6, clkhz));

    /* Run DCO. */
    PioDCOStart(&DCO);

    /* Set initial freq. */
    //*FIX* assert_(0 == PioDCOSetFreq(&DCO, GEN_FRQ_HZ, 0u));

    /* Run the main DCO algorithm. It spins forever. */
    PioDCOWorker2(&DCO);
}

void RAM (SpinnerMFSKTest)(void)
{
    uint32_t rndval = 77777777;
    for(;;)
    {
        /* This example sets new RND frequency every ~250 ms.
           Frequency shift is 5 Hz for each step.
        */
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ - 5*(rndval & 7), 0u);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(250);

        PRN32(&rndval);
    }
}

void RAM (SpinnerSweepTest)(void)
{
    int i = 0;
    for(;;)
    {
        /* This example sets new frequency every ~250 ms.
           Frequency shift is 5 Hz for each step.
        */
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ - 5*i, 0u);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);

        /* Return to initial freq after 20 steps (100 Hz). */
        if(++i == 20)
            i = 0;
    }
}

void RAM (SpinnerRTTYTest)(void)
{
    int32_t df = 170;   /* 170 Hz freq diff. */
    uint32_t rndval = 77777777;
    for(;;)
    {
        /* This example sets new PRN frequency every ~22 ms.
           Note: You should use precise timing while building a real transmitter.
        */
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ - df*(rndval & 1), 0u);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(22);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(22);

        PRN32(&rndval);
    }
}

void RAM (SpinnerMilliHertzTest)(void)
{
    int i = 0;
    for(;;)
    {
        /* This example sets new frequency every ~1s.
           Frequency shift is 0.99 Hz for each step.
        */
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ, 990*(++i&1));

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1000);
    }
}

void RAM (SpinnerWide4FSKTest)(void)
{
    int32_t df = 100;   /* 100 Hz freq diff * 4 = 400 Hz. */
    uint32_t rndval = 77777777;
    for(;;)
    {
        /* This example sets new PRN frequency every ~20 ms.
           Note: You should use precise timing while building a real transmitter.
        */
        PioDCOSetFreq(&DCO, GEN_FRQ_HZ - df*(rndval & 3), 0u);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(20);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(20);

        PRN32(&rndval);
    }
}

/* This example sets the OUT frequency to 5.555 MHz.
   Next every ~1 sec the shift of the OUT frequency relative to GPS
   reference is calculated and the OUT frequency is corrected.
   The example is working only when GPS receiver provides an
   accurate PPS output (pulse per second). If no such option,
   the correction parameter is zero.
*/
void RAM (SpinnerGPSreferenceTest)(void)
{
    const uint32_t ku32_freq = 5555000UL;
    const int kigps_pps_pin = 2;

    int32_t i32_compensation_millis = 0;

    GPStimeContext *pGPS = GPStimeInit(0, 9600, kigps_pps_pin);
    //*FIX* assert_(pGPS);
    DCO._pGPStime = pGPS;
    int tick = 0;
    for(;;)
    {
        PioDCOSetFreq(&DCO, ku32_freq, -2*i32_compensation_millis);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(2500);

        i32_compensation_millis = 
            PioDCOGetFreqShiftMilliHertz(&DCO, (uint64_t)(ku32_freq * 1000LL));

        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(2500);

        if(0 == ++tick % 6)
        {
            //stdio_set_driver_enabled(&stdio_uart, false);
            GPStimeDump(&(pGPS->_time_data));
            //stdio_set_driver_enabled(&stdio_uart, true);
        }
    }
}




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
