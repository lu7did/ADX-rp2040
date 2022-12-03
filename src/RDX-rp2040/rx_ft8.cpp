#include <Arduino.h>
#include "RDX-rp2040.h"
#include "constants.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * rx_ft8
 * Setup the ADC and start conversions
 * Code excerpts from
 * originally from ft8_lib by Karlis Goba (YL3JG)
 * excerpts taken from pi_ft8_xcvr by Godwin Duan (AA1GD) 2021
 * excerpts taken from Orange_Thunder by Pedro Colla (LU7DZ) 2018
 *
 * Adaptation to ADX-rp2040 project by Pedro Colla (LU7DZ) 2022
 * 
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "rx_ft8.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "decode_ft8.h"
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  Variables                                                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

const int CAPTURE_DEPTH = block_size;
int16_t fresh_signal[block_size] = {0};
uint dma_chan;
dma_channel_config cfg;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*  Setup and operate ADC                                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*---------------------------
 * setup_adc()
 * initial setup of the ADC conversion and DMA transfer
 */
void setup_adc(){
// Set up the adc

    return;

    _INFOLIST("%s started\n",__func__);
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC0 + ADC_CHANNEL);
    adc_init();
    adc_select_input(ADC_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(48000000 / sample_rate); //6khz sampling
    
    _INFOLIST("%s Arming DMA\n",__func__);
    sleep_ms(1000);

    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);
    _INFOLIST("%s completed\n",__func__);
}

/*-----------------
 * collect_adc()
 * periodically collect samples at the defined sample rate
 */
void collect_adc(){

        _INFOLIST("%s started\n",__func__);
        dma_channel_configure(dma_chan, &cfg,
            fresh_signal,    // pointer to where results are placed
            &adc_hw->fifo,  // src
            CAPTURE_DEPTH,  // transfer count
            true            // start immediately
        );
        adc_run(true);
        _INFOLIST("%s ADC running\n",__func__);
        
        // Once DMA finishes, stop any new conversions from starting, and clean up
        // the FIFO in case the ADC was still mid-conversion.
        //right here it's gonna wait for 160 ms. can run some things on this core while it's working
        dma_channel_wait_for_finish_blocking(dma_chan);
        adc_run(false);
        _INFOLIST("%s DMA Channel completed\n",__func__);
        adc_fifo_drain();
        _INFOLIST("%s DMA Channel drained\n",__func__);

}
