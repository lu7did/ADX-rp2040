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
#pragma once




#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#include "hardware/clocks.h"
#endif

//*-------------------------------------------------------------------------------------------------------------
//  Held structure to optimize system clock to achieve a target frequency
//  We can get closer to the derired frequency if we allow small adjustments
//  to the system clock. system clocks in the range 125 - 133 MHz are fast
//  enough to run the software. There are nearly 50 different frequencies in this
//  range, by chosing the frequency that gives the best match, we can get within about 4Khz.
//*-------------------------------------------------------------------------------------------------------------
struct PLLSettings{
  uint32_t frequency;
  uint8_t  refdiv;
  uint16_t fbdiv;
  uint8_t  postdiv1;
  uint8_t  postdiv2;
};
//*-------------------------------------------------------------------------------------------------------------
//* A list of all the achievable frequencies in range
//*-------------------------------------------------------------------------------------------------------------
PLLSettings possible_frequencies[] = {
      {125000000, 1, 125, 6, 2},
      {125142857, 1,  73, 7, 1},
      {125333333, 1,  94, 3, 3},
      {126000000, 1, 126, 6, 2},
      {126666666, 1,  95, 3, 3},
      {126857142, 1,  74, 7, 1},
      {127000000, 1, 127, 6, 2},
      {127200000, 1, 106, 5, 2},
      {127500000, 1,  85, 4, 2},
      {128000000, 1, 128, 6, 2},
      {128400000, 1, 107, 5, 2},
      {128571428, 1,  75, 7, 1},
      {129000000, 1, 129, 6, 2},
      {129333333, 1,  97, 3, 3},
      {129600000, 1, 108, 5, 2},
      {130000000, 1, 130, 6, 2},
      {130285714, 1,  76, 7, 1},
      {130500000, 1,  87, 4, 2},
      {130666666, 1,  98, 3, 3},
      {130800000, 1, 109, 5, 2},
      {131000000, 1, 131, 6, 2},
      {132000000, 1, 132, 6, 2},
      {133000000, 1, 133, 6, 2}
  };  
PLLSettings best_settings;
#define PLL_BASE      12000000
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                         Basic PIO firmware
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
struct PIOSettings{
  PIO       pio;
  uint      offset;
  uint32_t  sm;
};
#define PIO_WRAP_TARGET 0
#define PIO_WRAP        3
static const uint16_t PIO_program_instructions[] = {
            //     .wrap_target
    0xe000, //  0: set    pins, 0                    
    0xe001, //  1: set    pins, 1                    
    0xe003, //  2: set    pins, 3                    
    0xe002, //  3: set    pins, 2                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
//*-----------------------------------------------------------------------------------------------------
//* Structure to store PIO program
//*-----------------------------------------------------------------------------------------------------
static const struct pio_program PIO_program = {
    .instructions = PIO_program_instructions,
    .length = 4,
    .origin = -1,
};
//*-----------------------------------------------------------------------------------------------------
//* Obtain basic PIO stack configuration
//*-----------------------------------------------------------------------------------------------------
static inline pio_sm_config PIO_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + PIO_WRAP_TARGET, offset + PIO_WRAP);
    return c;
}

#endif
