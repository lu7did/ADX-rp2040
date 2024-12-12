//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ddsPIO                                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2024
// Version 1.0
// This is a rp2040 implementation of a digital clock in the range of 1 to 30 MHz
//*---------------------------------------------------------------------------------------------------------
//
// This implementation is largely based on the pico-hf-oscillator project by
//
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://www.qrz.com/db/r2bdy
//
//*---------------------------------------------------------------------------------------------------------
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
//*-----------------------------------------------------------------------------------
//
//  PLATFORM
//      Raspberry Pi pico.
//
//  REVISION HISTORY (original sequence by Roman Piksaykin)
// 
//      Rev 0.1   05 Nov 2023   Initial release
//      Rev 0.2   18 Nov 2023
//      Rev 1.0   10 Dec 2023   Improved frequency range (to ~33.333 MHz).
//
//  Ported to the Arduino IDE environment by Dr. Pedro E. Colla (LU7DZ)
//
//      https://github.com/lu7did/ADX-rp2040/src/test/ddsPIO
//
//  LICENCE
//      MIT License (http://www.opensource.org/licenses/mit-license.php)
//
//  Copyright (c) 2023 by Roman Piksaykin
//  Copyright (c) 2924 by Dr. Pedro E. Colla (only Arduino IDE porting segments)
//  
//*-----------------------------------------------------------------------------------
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
//*-----------------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include "./defines.h"
#include "dco2.pio.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdio/driver.h"
#include "./lib/assert.h"
#include "./debug/logutils.h"
#include "hwdefs.h"
#include "protos.h"
#include <string.h>
#include "./gpstime/GPStime.h"
#include "ddsPIO.h"

//*---------------------------------------------------------------------------------------------------------
PioDco DCO; /* External in order to access in both cores. */
char hi[80];
uint32_t f=GEN_FRQ_HZ;
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                      Setup & initialization
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void setup() { 

    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;
    set_sys_clock_khz(clkhz / 1000L, true);

    Serial.begin(115200);
    while (!Serial);
    
    _INFOLIST("%s Version(%s) Build(%s)\n",PROGNAME,VERSION,BUILD);
    _INFOLIST("%s Default clock %ld KHz\n",__func__,f);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                      Execution Loop
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

void loop() {

        if (Serial.available() != 0) {
            int dataIn = Serial.parseInt();    // only numbers, frequency in Hz, without any kind of points, cr/lf or /n  
                                              // for example 7060000  tunes to 7.060,00 kHz 
            uint32_t fnew=dataIn*1000L;
            if (fnew != f) {
              f=fnew;
              _INFOLIST("%s Keyboard entry (%d) KHz, new clock %ld Hz\n",__func__,dataIn,f);
              PioDCOSetFreq(&DCO, f, 0u);           
              gpio_set_mask(1<<LED_BUILTIN);
              delay(250);
              gpio_clr_mask(1<<LED_BUILTIN);   

            }

        }
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                    2nd Core Process
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void core1_entry()
{
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;

    /* Initialize DCO */
    PioDCOInit(&DCO, 6, clkhz);

    /* Run DCO. */
    PioDCOStart(&DCO);

    /* Set initial freq. */
    PioDCOSetFreq(&DCO, f, 0u);

    /* Run the main DCO algorithm. It spins forever. */
    PioDCOWorker2(&DCO);
}

void RAM (SpinnerRTTYTest)(void)
{
    for(;;)
    {
        /* This example sets new PRN frequency every ~22 ms.
           Note: You should use precise timing while building a real transmitter.
        */
        //PioDCOSetFreq(&DCO, GEN_FRQ_HZ - df*(rndval & 1), 0u);
        PioDCOSetFreq(&DCO, f, 0u);

        /* LED signal */
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(22);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(22);

    }
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                    PIO Interface & DCO Implementation
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
volatile int32_t si32precise_cycles;

//*---------------------------------------------------------------------------------------------------------
/// @brief Initializes DCO context and prepares PIO hardware.
/// @param pdco Ptr to DCO context.
/// @param gpio The GPIO of DCO output.
/// @param cpuclkhz The system CPU clock freq., Hz.
/// @return 0 if OK.
//*---------------------------------------------------------------------------------------------------------
int PioDCOInit(PioDco *pdco, int gpio, int cpuclkhz)
{
    memset(pdco, 0, sizeof(PioDco));

    pdco->_clkfreq_hz = cpuclkhz;
    pdco->_pio = pio0;
    pdco->_gpio = gpio;
    pdco->_offset = pio_add_program(pdco->_pio, &dco_program);
    pdco->_ism = pio_claim_unused_sm(pdco->_pio, true);

    gpio_init(pdco->_gpio);
    pio_gpio_init(pdco->_pio, pdco->_gpio);

    dco_program_init(pdco->_pio, pdco->_ism, pdco->_offset, pdco->_gpio);
    pdco->_pio_sm = dco_program_get_default_config(pdco->_offset);

    sm_config_set_out_shift(&pdco->_pio_sm, true, true, 32);           // Autopull.
    sm_config_set_fifo_join(&pdco->_pio_sm, PIO_FIFO_JOIN_TX);
    sm_config_set_set_pins(&pdco->_pio_sm, pdco->_gpio, 1);
    
    pio_sm_init(pdco->_pio, pdco->_ism, pdco->_offset, &pdco->_pio_sm);

    return 0;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Sets DCO working frequency in Hz: Fout = ui32_frq_hz + ui32_frq_millihz * 1e-3.
/// @param pdco Ptr to DCO context.
/// @param i32_frq_hz The `coarse` part of frequency [Hz]. Might be negative.
/// @param ui32_frq_millihz The `fine` part of frequency [Hz].
/// @return 0 if OK. -1 invalid freq.
/// @attention The func can be called while DCO running.
//*---------------------------------------------------------------------------------------------------------

int PioDCOSetFreq(PioDco *pdco, uint32_t ui32_frq_hz, int32_t ui32_frq_millihz)
{
    /* RPix: Calculate an accurate value of phase increment of the freq 
       per 1 tick of CPU clock, here 2^24 is scaling coefficient. */

    const int64_t i64denominator = 2000LL * (int64_t)ui32_frq_hz + (int64_t)ui32_frq_millihz;
    pdco->_frq_cycles_per_pi = (int32_t)(((int64_t)pdco->_clkfreq_hz * (int64_t)(1<<24) * 1000LL
                                         +(i64denominator>>1)) / i64denominator);

    si32precise_cycles = pdco->_frq_cycles_per_pi - (PIOASM_DELAY_CYCLES<<24);

    pdco->_ui32_frq_hz = ui32_frq_hz;
    pdco->_ui32_frq_millihz = ui32_frq_millihz;

    return 0;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Obtains the frequency shift [milliHz] which is calculated for a given frequency.
/// @param pdco Ptr to Context.
/// @param u64_desired_frq_millihz The frequency for which we want to calculate correction.
/// @return The value of correction we need to subtract from desired freq. to compensate
/// @return Pico's reference clock shift. 2854974.
//*---------------------------------------------------------------------------------------------------------
int32_t PioDCOGetFreqShiftMilliHertz(const PioDco *pdco, uint64_t u64_desired_frq_millihz)
{

/*--------------------------------------------
  No GPS correction in this porting
 */

    return 0U;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Starts the DCO.
/// @param pdco Ptr to DCO context.
//*---------------------------------------------------------------------------------------------------------
void PioDCOStart(PioDco *pdco)
{
    pio_sm_set_enabled(pdco->_pio, pdco->_ism, true);
    pdco->_is_enabled = YES;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Stops the DCO.
/// @param pdco Ptr to DCO context.
//*---------------------------------------------------------------------------------------------------------
void PioDCOStop(PioDco *pdco)
{
    pio_sm_set_enabled(pdco->_pio, pdco->_ism, false);
    pdco->_is_enabled = NO;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Main worker task of DCO V.2. It is time critical, so it ought to be run on
/// @brief the dedicated pi pico core.
/// @param pDCO Ptr to DCO context.
/// @return No return. It spins forever.
//*---------------------------------------------------------------------------------------------------------
void RAM (PioDCOWorker2)(PioDco *pDCO)
{
    PIO pio = pDCO->_pio;
    uint sm = pDCO->_ism;
    int32_t i32acc_error = 0;
    uint32_t i32wc, i32reg;

Worker2:
    i32reg = si32precise_cycles;
    i32wc = (i32reg - i32acc_error) >> 24U;
    pio_sm_put_blocking(pio, sm, i32wc);
    i32acc_error += (i32wc << 24U) - i32reg;
    
    goto Worker2;
}
//*---------------------------------------------------------------------------------------------------------
/// @brief Sets DCO running mode.
/// @param pdco Ptr to DCO context.
/// @param emode Desired mode.
/// @attention Not actual so far. User-independent freq. correction not impl'd yet. !FIXME!
//*---------------------------------------------------------------------------------------------------------
void PioDCOSetMode(PioDco *pdco, enum PioDcoMode emode)
{
    pdco->_mode = emode;
    switch(emode)
    {
        case eDCOMODE_IDLE:
            PioDCOStop(pdco);
            break;

        case eDCOMODE_GPS_COMPENSATED:
            PioDCOStart(pdco);
            break;

        default:
            break;
    }
}
