//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              dds_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2024
// Version 1.0
// This is a rp2040 implementation of a quadrature oscilator in the range of 1 to 30 MHz
//*-------------------------------------------------------------------------------------------------------------
// Based on 
// https://github.com/lu9da/quadrature_oscillator_pio 
// by
// Ricardo Suarez (LU9DA), 2024
// https://lu9da.org
//*-------------------------------------------------------------------------------------------------------------
// Similar quadrature oscillator implementation
// https://hackaday.io/project/192311-pi-pico-rx
// https://github.com/dawsonjon/PicoRX
// by 
// Jon Dawson
//*-------------------------------------------------------------------------------------------------------------
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
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
//*-------------------------------------------------------------------------------------------------------------
//
// Original license required by Ricardo Suarez to be placed on any use
//
// generador por pio de frecuencia en cuadratura, para exitar directamente un mixer tayloe
// tipo softrock (ahorra un SI5351 para generar I-lo e Q-lo) de 0 a 30 mhz
// Encontrado por casulidad por ahi y adaptado para mis necesidades a Arduino RP2040
// Creditos a quien corresponda, lamentablemente no pude encontrar de nuevo la pagina 
// de donde lo baje.
// anda en un solo core, pero lo probe con los dos cores y funciona perfectamente tambien
// las salidas en cuadratura estan en GPIO0 y GPIO1
// - adaptado y modificado en 2024 por LU9DA -
// Uso publico, si se comparte o usa en proyectos, este encabezado DEBE PERMANECER como esta
// Public use, if shared or used in projects, this header MUST REMAIN as is
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              dds_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//
//
//               *----*    *----*
//    (GPIO0)    |    |    |    |
//               |    |    |    |
//           *---*    *----*    *--
//
//                  *----*    *----*
//    (GPIO1)       |    |    |    |
//                  |    |    |    |
//            *-----*    *----*    *--
//             
//     GPIO0    0  1  1  0  0 
//     GPIO1    0  0  1  1  0
//
//  An internal finite state automata implemented using PIO would cycle the GPIO0/GPIO1 thru the sequence
//  {0x00,0x01,0x02,0x03} as quick as possible
//  The actual transition speed would be defined by the PIO clock which in turn is derived from the system
//  clock using a number of divisors, so change the dividers frequencies in the range 1-30 MHz can be 
//  obtained.
//  As the dividers perform integer math no precise frequency match can be achieved, however the system 
//  clock can be changed to reduce the error (difference) with the desired frequency, therefore each time
//  a frequency change is made an inspection is performed to identify which system clock frequency will reduce
//  the error to a minimum.
//*-------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
//*-------------------------------------------------------------------------------------------------------------
//*
//*-------------------------------------------------------------------------------------------------------------
#include "dds_rp2040.h"
//*-------------------------------------------------------------------------------------------------------------
#define PROGRAM  "dds_rp2040"
#define VERSION  "1.0"
#define BUILD    "0.0"
#define AUTHOR   "Dr. Pedro E. Colla (LU7DZ)"
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                    Development support structures
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
char hi[128];

#define DEBUG   1
#define _SERIAL Serial
#ifdef DEBUG
#define _INFOLIST(...) \
  do { \
    strcpy(hi,"@"); \
    sprintf(hi+1,__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)
#else //!DEBUG
#define _INFOLIST(...) (void)0
#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

double fx=14000000;      //Initial frequency setup
PIOSettings pio;         //PIO structure

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         Quadrature DDS Management functions
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*-----------------------------------------------------------------------------------------------------
//* This procedure loads the PIO firmware and triggers the execution, this function is re-entrable and
//* re-usable, therefore to change the frequency it the PIO firmware needs to be just booted again
//* a small jitter would be assumed as Ok
//*-----------------------------------------------------------------------------------------------------
static inline float PIO_program_init(PIO pio, uint sm, uint offset, float tuned_frequency) {

    pio_sm_config c = PIO_program_get_default_config(offset);
/*--------------    
  Map the state machine's OUT pin group to one pin, namely the `pin`
  parameter to this function.
*/  
    sm_config_set_set_pins(&c, 0, 2);

/*---------------
  Set GPIO0 and GPIO1 as the output pin, connect the PIO processor to the pad 
  define pin strength, slew rate and pin direction
*/  
    pio_gpio_init(pio, 0);
    pio_gpio_init(pio, 1);

    gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(1, GPIO_DRIVE_STRENGTH_2MA);
    
    gpio_set_slew_rate(0, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(1, GPIO_SLEW_RATE_SLOW);
    
    pio_sm_set_consecutive_pindirs(pio, sm, 0, 2, true);

/*---------------
  Compute the best adjusted frequency for a given frequency specified
  and obtain the system clock that best suit that requirement
*/      
    
    float adjusted_frequency_up = tuned_frequency; //+ 6000;
    float adjusted_frequency_down = tuned_frequency; //- 6000;
    float best_frequency = 1;
    float best_divider;
    float best_error=1000000;
    best_frequency = 1;

    for(uint8_t idx = 0; idx < sizeof(possible_frequencies)/sizeof(PLLSettings); idx++)
    {
      float system_clock_frequency = possible_frequencies[idx].frequency;
      float ideal_divider = system_clock_frequency/(4.0f*adjusted_frequency_up);
      float nearest_divider = round(256.0f*ideal_divider)/256.0f;
      float actual_frequency = system_clock_frequency/nearest_divider;
      float error = abs(actual_frequency - 4.0f*adjusted_frequency_up);
      if(error < best_error)
      {
        best_frequency = actual_frequency;
        best_settings = possible_frequencies[idx];
        best_divider = nearest_divider;
        best_error = error;
      }

      //sprintf(hi,"Try(%f MHz) ideal(%f) nearest(%f) actual_frequency(%f) error(%f) best_error(%f)\n",system_clock_frequency,ideal_divider,nearest_divider,actual_frequency,error,best_error);
      //Serial.print(hi);

      ideal_divider = system_clock_frequency/(4.0f*adjusted_frequency_down);
      nearest_divider = round(256.0f*ideal_divider)/256.0f;
      actual_frequency = system_clock_frequency/nearest_divider;
      error = abs(actual_frequency - 4.0f*adjusted_frequency_down);
      if(error < best_error)
      {
        best_frequency = actual_frequency;
        best_settings = possible_frequencies[idx];
        best_divider = nearest_divider;
        best_error = error;
      }
    }

/*---------------------------
  Once obtained the best system clock for a given frequency set it
*/
  uint32_t vco_freq = (PLL_BASE / best_settings.refdiv) * best_settings.fbdiv;
  set_sys_clock_pll(vco_freq, best_settings.postdiv1, best_settings.postdiv2);

/*----------------------------
   Set the matching PIO divider
*/   
    sm_config_set_clkdiv(&c, best_divider);

/*----------------------------
   Load the PIO firmware and configuration, kick the PIO firmwar
*/   
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

/*-----------------------------
   Return the set frequency to the caller
*/   
    return best_frequency/4.0f;
}
//*-----------------------------------------------------------------------------------------------------
//* This procedure set the frequency and compute the offset
//*-----------------------------------------------------------------------------------------------------
void PIO_setFrequency(double fQ) {

  double fnco = PIO_program_init(pio.pio, pio.sm, pio.offset, fQ);
  double offset_frequency_Hz = fQ - fnco;
  _INFOLIST("%s Freq(%f Hz) NCO(%f Hz) offset(%f Hz) \n",__func__,fQ,fnco,offset_frequency_Hz);

}  
//*-----------------------------------------------------------------------------------------------------
//* This procedure initially configure the PIO environment
//*-----------------------------------------------------------------------------------------------------
void PIO_configure() {

  pio.pio = pio0;
  pio.offset = pio_add_program(pio.pio, &PIO_program);
  pio.sm = pio_claim_unused_sm(pio.pio, true);
  PIO_program_get_default_config(pio.offset);
  _INFOLIST("%s PIO configuration completed Ok\n",__func__);

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         Setup function
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

void setup() {

/*------------------------------------
  Setup default LED
*/
 pinMode(LED_BUILTIN, OUTPUT);
 gpio_set_mask(1 << LED_BUILTIN);

/*------------------------------------
  Setup serial port, flash LED whild not opening the port
*/
  
 Serial.begin(115200);
 while (!Serial) {  //wait for PC USB-serial to open
  
  gpio_set_mask(1<<LED_BUILTIN);
  delay(250);
  gpio_clr_mask(1<<LED_BUILTIN);
  delay(250);
 }
 _INFOLIST("%s Firmware %s version %s build(%s)\n",__func__,PROGRAM,VERSION,BUILD);

/*------------------------------------
  Configure PIO and set initial frequency
*/
 PIO_configure();
 PIO_setFrequency(fx);

/*------------------------------------
  Completed setup
*/
_INFOLIST("%s Setup completed!\n",__func__);
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                         Loop function
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
void loop() {

  if (Serial.available() != 0)
  {
    int dataIn = Serial.parseInt();    // only numbers, frequency in Hz, without any kind of points, cr/lf or /n  
                                       // for example 7060000  tunes to 7.060,00 kHz 

    _INFOLIST("%s Keyboard entry (%d) Hz\n",__func__,dataIn);                                   
    fx = dataIn;
    PIO_setFrequency(fx);

    gpio_set_mask(1<<LED_BUILTIN);
    delay(250);
    gpio_clr_mask(1<<LED_BUILTIN);
    delay(250);

  }
  delay(1);
 
}
