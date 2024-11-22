//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              dds_rp2040                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2024
// Version 1.0
// This is a rp2040 implementation of a quadrature oscilator in the range of 1 to 30 MHz
//*-------------------------------------------------------------------------------------------------------------
#pragma once

#define OVERCLOCK 1                  //Users extended clock range to improve PLL error at high frequencies

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
{125142857, 1, 73, 7, 1},
{125333333, 1, 94, 3, 3},
{126000000, 1, 126, 6, 2},
{126666666, 1, 95, 3, 3},
{126857142, 1, 74, 7, 1},
{127000000, 1, 127, 6, 2},
{127200000, 1, 106, 5, 2},
{127500000, 1, 85, 4, 2},
{128000000, 1, 128, 6, 2},
{128400000, 1, 107, 5, 2},
{128571428, 1, 75, 7, 1},
{129000000, 1, 129, 6, 2},
{129333333, 1, 97, 3, 3},
{129600000, 1, 108, 5, 2},
{130000000, 1, 130, 6, 2},
{130285714, 1, 76, 7, 1},
{130500000, 1, 87, 4, 2},
{130666666, 1, 98, 3, 3},
{130800000, 1, 109, 5, 2},
{131000000, 1, 131, 6, 2},
{132000000, 1, 132, 6, 2},
{133000000, 1, 133, 6, 2},
{133200000, 1, 111, 5, 2},
{133333333, 1, 100, 3, 3}

#ifdef OVERCLOCK      

,{133500000, 1, 89, 4, 2},
{133714285, 1, 78, 7, 1},
{134000000, 1, 67, 6, 1},
{134400000, 1, 112, 5, 2},
{134666666, 1, 101, 3, 3},
{135000000, 1, 90, 4, 2},
{135428571, 1, 79, 7, 1},
{135600000, 1, 113, 5, 2},
{136000000, 1, 102, 3, 3},
{136500000, 1, 91, 4, 2},
{136800000, 1, 114, 5, 2},
{137142857, 1, 80, 7, 1},
{137333333, 1, 103, 3, 3},
{138000000, 1, 115, 5, 2},
{138666666, 1, 104, 3, 3},
{138857142, 1, 81, 7, 1},
{139200000, 1, 116, 5, 2},
{139500000, 1, 93, 4, 2},
{140000000, 1, 105, 3, 3},
{140400000, 1, 117, 5, 2},
{140571428, 1, 82, 7, 1},
{141000000, 1, 94, 4, 2},
{141333333, 1, 106, 3, 3},
{141600000, 1, 118, 5, 2},
{142000000, 1, 71, 6, 1},
{142285714, 1, 83, 7, 1},
{142500000, 1, 95, 4, 2},
{142666666, 1, 107, 3, 3},
{142800000, 1, 119, 5, 2},
{144000000, 1, 120, 5, 2},
{145200000, 1, 121, 5, 2},
{145333333, 1, 109, 3, 3},
{145500000, 1, 97, 4, 2},
{145714285, 1, 85, 7, 1},
{146000000, 1, 73, 6, 1},
{146400000, 1, 122, 5, 2},
{146666666, 1, 110, 3, 3},
{147000000, 1, 98, 4, 2},
{147428571, 1, 86, 7, 1},
{147600000, 1, 123, 5, 2},
{148000000, 1, 111, 3, 3},
{148500000, 1, 99, 4, 2},
{148800000, 1, 124, 5, 2},
{149142857, 1, 87, 7, 1},
{149333333, 1, 112, 3, 3},
{150000000, 1, 125, 5, 2},
{150666666, 1, 113, 3, 3},
{150857142, 1, 88, 7, 1},
{151200000, 1, 126, 5, 2},
{151500000, 1, 101, 4, 2},
{152000000, 1, 114, 3, 3},
{152400000, 1, 127, 5, 2},
{152571428, 1, 89, 7, 1},
{153000000, 1, 102, 4, 2},
{153333333, 1, 115, 3, 3},
{153600000, 1, 128, 5, 2},
{154000000, 1, 77, 6, 1},
{154285714, 1, 90, 7, 1},
{154500000, 1, 103, 4, 2},
{154666666, 1, 116, 3, 3},
{154800000, 1, 129, 5, 2},
{156000000, 1, 130, 5, 2},
{157200000, 1, 131, 5, 2},
{157333333, 1, 118, 3, 3},
{157500000, 1, 105, 4, 2},
{157714285, 1, 92, 7, 1},
{158000000, 1, 79, 6, 1},
{158400000, 1, 132, 5, 2},
{158666666, 1, 119, 3, 3},
{159000000, 1, 106, 4, 2},
{159428571, 1, 93, 7, 1},
{159600000, 1, 133, 5, 2},
{160000000, 1, 120, 3, 3},
{160500000, 1, 107, 4, 2},
{160800000, 1, 67, 5, 1},
{161142857, 1, 94, 7, 1},
{161333333, 1, 121, 3, 3},
{162000000, 1, 108, 4, 2},
{162666666, 1, 122, 3, 3},
{162857142, 1, 95, 7, 1},
{163200000, 1, 68, 5, 1},
{163500000, 1, 109, 4, 2},
{164000000, 1, 123, 3, 3},
{164571428, 1, 96, 7, 1},
{165000000, 1, 110, 4, 2},
{165333333, 1, 124, 3, 3},
{165600000, 1, 69, 5, 1},
{166000000, 1, 83, 6, 1},
{166285714, 1, 97, 7, 1},
{166500000, 1, 111, 4, 2},
{166666666, 1, 125, 3, 3},
{168000000, 1, 126, 3, 3},
{169333333, 1, 127, 3, 3},
{169500000, 1, 113, 4, 2},
{169714285, 1, 99, 7, 1},
{170000000, 1, 85, 6, 1},
{170400000, 1, 71, 5, 1},
{170666666, 1, 128, 3, 3},
{171000000, 1, 114, 4, 2},
{171428571, 1, 100, 7, 1},
{172000000, 1, 129, 3, 3},
{172500000, 1, 115, 4, 2},
{172800000, 1, 72, 5, 1},
{173142857, 1, 101, 7, 1},
{173333333, 1, 130, 3, 3},
{174000000, 1, 116, 4, 2},
{174666666, 1, 131, 3, 3},
{174857142, 1, 102, 7, 1},
{175200000, 1, 73, 5, 1},
{175500000, 1, 117, 4, 2},
{176000000, 1, 132, 3, 3},
{176571428, 1, 103, 7, 1},
{177000000, 1, 118, 4, 2},
{177333333, 1, 133, 3, 3},
{177600000, 1, 74, 5, 1},
{178000000, 1, 89, 6, 1},
{178285714, 1, 104, 7, 1},
{178500000, 1, 119, 4, 2},
{180000000, 1, 120, 4, 2},
{181500000, 1, 121, 4, 2},
{181714285, 1, 106, 7, 1},
{182000000, 1, 91, 6, 1},
{182400000, 1, 76, 5, 1},
{183000000, 1, 122, 4, 2},
{183428571, 1, 107, 7, 1},
{184000000, 1, 92, 6, 1},
{184500000, 1, 123, 4, 2},
{184800000, 1, 77, 5, 1},
{185142857, 1, 108, 7, 1},
{186000000, 1, 124, 4, 2},
{186857142, 1, 109, 7, 1},
{187200000, 1, 78, 5, 1},
{187500000, 1, 125, 4, 2},
{188000000, 1, 94, 6, 1},
{188571428, 1, 110, 7, 1},
{189000000, 1, 126, 4, 2},
{189600000, 1, 79, 5, 1},
{190000000, 1, 95, 6, 1},
{190285714, 1, 111, 7, 1},
{190500000, 1, 127, 4, 2},
{192000000, 1, 128, 4, 2},
{193500000, 1, 129, 4, 2},
{193714285, 1, 113, 7, 1},
{194000000, 1, 97, 6, 1},
{194400000, 1, 81, 5, 1},
{195000000, 1, 130, 4, 2},
{195428571, 1, 114, 7, 1},
{196000000, 1, 98, 6, 1},
{196500000, 1, 131, 4, 2},
{196800000, 1, 82, 5, 1},
{197142857, 1, 115, 7, 1},
{198000000, 1, 132, 4, 2},
{198857142, 1, 116, 7, 1},
{199200000, 1, 83, 5, 1},
{199500000, 1, 133, 4, 2},
{200000000, 1, 100, 6, 1},
{200571428, 1, 117, 7, 1},
{201000000, 1, 67, 4, 1},
{201600000, 1, 84, 5, 1},
{202000000, 1, 101, 6, 1},
{202285714, 1, 118, 7, 1},
{204000000, 1, 119, 7, 1},
{205714285, 1, 120, 7, 1},
{206000000, 1, 103, 6, 1},
{206400000, 1, 86, 5, 1},
{207000000, 1, 69, 4, 1},
{207428571, 1, 121, 7, 1},
{208000000, 1, 104, 6, 1},
{208800000, 1, 87, 5, 1},
{209142857, 1, 122, 7, 1},
{210000000, 1, 105, 6, 1},
{210857142, 1, 123, 7, 1},
{211200000, 1, 88, 5, 1},
{212000000, 1, 106, 6, 1},
{212571428, 1, 124, 7, 1},
{213000000, 1, 71, 4, 1},
{213600000, 1, 89, 5, 1},
{214000000, 1, 107, 6, 1},
{214285714, 1, 125, 7, 1},
{216000000, 1, 126, 7, 1},
{217714285, 1, 127, 7, 1},
{218000000, 1, 109, 6, 1},
{218400000, 1, 91, 5, 1},
{219000000, 1, 73, 4, 1},
{219428571, 1, 128, 7, 1},
{220000000, 1, 110, 6, 1},
{220800000, 1, 92, 5, 1},
{221142857, 1, 129, 7, 1},
{222000000, 1, 111, 6, 1},
{222857142, 1, 130, 7, 1},
{223200000, 1, 93, 5, 1},
{224000000, 1, 112, 6, 1},
{224571428, 1, 131, 7, 1},
{225000000, 1, 75, 4, 1},
{225600000, 1, 94, 5, 1},
{226000000, 1, 113, 6, 1},
{226285714, 1, 132, 7, 1},
{228000000, 1, 133, 7, 1},
{230000000, 1, 115, 6, 1},
{230400000, 1, 96, 5, 1},
{231000000, 1, 77, 4, 1},
{232000000, 1, 116, 6, 1},
{232800000, 1, 97, 5, 1}
#endif //OVERCLOCK      
};  

/*
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
#ifdef OVERCLOCK      
      //*---
     ,{134000000, 1, 67, 6, 1},
      {134400000, 1, 112, 5, 2},
      {134666666, 1, 101, 3, 3},
      {135000000, 1, 90, 4, 2},
      {135428571, 1, 79, 7, 1},
      {135600000, 1, 113, 5, 2},
      {136000000, 1, 102, 3, 3},
      {136500000, 1, 91, 4, 2},
      {136800000, 1, 114, 5, 2},
      {137142857, 1, 80, 7, 1},
      {137333333, 1, 103, 3, 3},
      {138000000, 1, 115, 5, 2},
      {138666666, 1, 104, 3, 3},
      {138857142, 1, 81, 7, 1},
      {139200000, 1, 116, 5, 2},
      {139500000, 1, 93, 4, 2},
      {140000000, 1, 105, 3, 3},
      {140400000, 1, 117, 5, 2},
      {140571428, 1, 82, 7, 1},
      {141000000, 1, 94, 4, 2},
      {141333333, 1, 106, 3, 3},
      {141600000, 1, 118, 5, 2},
      {142000000, 1, 71, 6, 1},
      {142285714, 1, 83, 7, 1},
      {142500000, 1, 95, 4, 2},
      {142666666, 1, 107, 3, 3},
      {142800000, 1, 119, 5, 2},
      {144000000, 1, 120, 5, 2},
      {145200000, 1, 121, 5, 2},
      {145333333, 1, 109, 3, 3},
      {145500000, 1, 97, 4, 2},
      {145714285, 1, 85, 7, 1},
      {146000000, 1, 73, 6, 1},
      {146400000, 1, 122, 5, 2},
      {146666666, 1, 110, 3, 3},
      {147000000, 1, 98, 4, 2},
      {147428571, 1, 86, 7, 1},
      {147600000, 1, 123, 5, 2},
      {148000000, 1, 111, 3, 3},
      {148500000, 1, 99, 4, 2},
      {148800000, 1, 124, 5, 2},
      {149142857, 1, 87, 7, 1},
      {149333333, 1, 112, 3, 3},
      {150000000, 1, 125, 5, 2}
#endif //OVERCLOCK      
};  
*/
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

#include "pio.h"
/*
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
*/