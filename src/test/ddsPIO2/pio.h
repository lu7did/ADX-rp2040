#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ---------- //
// squarewave //
// ---------- //

#define PIO_WRAP_TARGET 0
#define PIO_WRAP 3

static const uint16_t PIO_program_instructions[] = {
            //     .wrap_target
    0xe081, //  0: set    pindirs, 1                 
    0xe101, //  1: set    pins, 1                [1] 
    0xe000, //  2: set    pins, 0                    
    0x0001, //  3: jmp    1                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program PIO_program = {
    .instructions = PIO_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config PIO_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + PIO_WRAP_TARGET, offset + PIO_WRAP);
    return c;
}
#endif

