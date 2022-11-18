#include <Arduino.h>
//#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                         PIO FREQUENCY COUNTER IMPLEMENTATION                                *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
/*----------------------------------------------------------------------------------------------*
 * A PIO state machine is configured to handle the signal edge detection and trigger a IRQ which*
 * is handled by a low overhead ISR on core1. This has the advantage over the ZCD and ADCZ      *
 * algorithms to freed the core2 for other purposes of future implementation (specially digital *
 * signal processing for reception).                                                            *
 * The rising events are used to mark ticks on the uSec system timer so the frequency can be    *
 * computed with a resolution of +/- 2 uSec plus the overhead to process the interrupt which    *
 * should yield a frequency yield more precise                                                  *
 * ============================================================================================ *
 * Code inspired on simply_isr by Daniel Garcia Briceno                                         *
 * Code modified, PIO assembler made and evaluation performed by Dr. Pedro E. Colla (LU7DZ)     *
 */

#include "freqPIO.pio.h"
#include "pico.h"
#include "hardware/structs/pio.h"

/**
 * These defines represent each state machine.
 * The value is the bit in the IRQ register that
 * will be set by each state machine thanks to "irq wait 0 rel"
 */
#define PIO_SM_0_IRQ 0b0001
#define PIO_SM_1_IRQ 0b0010
#define PIO_SM_2_IRQ 0b0100
#define PIO_SM_3_IRQ 0b1000

/**
 * This variable will shadow the IRQ flags set by the PIO state machines.
 * Typically you do not want to do work in ISRs because the main thread
 * has more important things to do. Because of that, when we get the ISR
 * I'm simply going to copy the state machine that fired the ISR into
 * this variable.
 *
 * Variable is volatile so that it doesn't get cached in a CPU register
 * in the main thread. Without this it's possible that you never see
 * irq_flags get set even though the ISR is firing all the time.
 *
 * Of course, you can really do whatever you want in the ISR, it's up to you.
 */
volatile uint32_t   irq_flags    = 0;
volatile uint32_t   t_previous;
volatile uint32_t   t_current;
volatile uint32_t   period;
bool pioirq=false;
#define FSKpin      27      //Frequency counter algorithm, signal input PIN



/**
 * This function is called when the IRQ is fired by the state machine.
 * @note See enable_pio_isrs for how to register this function to be called
 */
void IRQ_handler() {
  
    // Read the IRQ register to get the IRQ flags from the state machine
    // This tells me which state machine sent the IRQ
/*--------    
 *  Read the IRQ register to get the IRQ flags from the state machine
 *  which inform which state machine sent the IRQ. This is pure paranoid
 *  level defensive programming as We've fired only pio0.
 */
    irq_flags = pio0_hw->irq;

 /*--------   
  * IRQ_OFFSET is written a 1 to clear, so this acknowledge the PIO
  * firmware that the interrupt has been served and unblock the
  * running. This is important as the PIO is running a time critical
  * function and needs to start waiting for the signal to clear with
  * a fall and start looking for the next rise
  */
    
    hw_clear_bits(&pio0_hw->irq, irq_flags);

/*--------
 * Mark the current timestamp, compare with previous and get the difference
 * as the ticks have a 1 uSec resolution the measurement will have a maximum
 * of +/- 2 uSec (worst case) error allowed
 */
    t_current = time_us_32();
    period=t_current-t_previous;
    t_previous=t_current;
    pioirq=true;
   
}

/**
 * Lets the pico know that we want it to notify us of the PIO ISRs.
 * @note in freqPIO.pio we enable irq0. This tells the state machine
 *       to send the ISRs to the core, we still need to tell the core
 *       to send them to our program.
 */
void enable_pio_isrs() {
    // Set the function that will be called when the PIO IRQ comes in.

/*-------
 * define the local IRQ handler for interrupts fired by the PIO firmware
 * the interrupt is honored allowing the firmware to flow despite if the
 * interrupt is used or not.
 * IMPORTANT NOTE from the original program simply_isr
 * "The docs says if an IRQ comes an there is not a handler for it will 
 * work like a breakpoint, which seems bad". This won't happen here
 */
    irq_set_exclusive_handler(PIO0_IRQ_0, IRQ_handler);   
    irq_set_enabled(PIO0_IRQ_0, true);
}

/**
 * Loads freqPIO.pio program into PIO memory
 */
void load_pio_programs() {

/*----------    
 * Define the pio0 PIO state machine to be used, load the firmware
 * to run on it, provide the FSK pin defined at PXD system level to
 * be the signal input as the pin to watch and interrupt with
 */
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &freqPIO_program);   
    freqPIO_program_init(pio, 0, offset,FSKpin);
}
/*-------------------------------------------------------------------------*
 * PIO_init()
 * Main initialization, called from PDX system setup if the FSK_FREQPIO
 * mode is enabled
 */
void PIO_init() {

/*-----------------------------
 * Load freqPIO.pio microcode for the PIO sequential machine into memory
 * then enable the PIO to start firing IRQs to the system program
 */
    load_pio_programs();
    pioirq=false;
    enable_pio_isrs();
}
