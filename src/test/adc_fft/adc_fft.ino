// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)
// Modified for the RDX project by Pedro E. Colla (LU7DZ) 2022

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fftr.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <si5351.h>
#include "hardware/watchdog.h"
#include "Wire.h"
#include <EEPROM.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include <WiFi.h>
#include <Time.h>
#include <stdbool.h>

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
 * Sampling parameters
 * ADC Clock=48 MHz
 * The sample acquisition time is 2 uSecs thus the maximum divisor is 96
 * therefore the maximum sampling rate is 500000 per second or 500 KSamples/sec
 * The value of CLOCK_DIV will define the actual sample rate in this program
 * a value of 0 equals using 96 as the divider because the restriction will come 
 * from the ADC conversion rate
 *    
 * 0     = 500,000 Hz
 * 960   = 50,000 Hz
 * 9600  = 5,000 Hz
 * 
  */
#define ADC_CLOCK 48000000  
#define CLOCK_DIV 9600
#define FSAMP (ADC_CLOCK / CLOCK_DIV)


// Channel 0 is GPIO26
#define ADC_PIN   26
#define CAPTURE_CHANNEL 0

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 1000

/*------------
 * DMA definitions
 * needs to be global se it is used from various places
 */
dma_channel_config cfg;
uint dma_chan;
/*----------
 * Table to convert from energy bin index to frequency
 */
float freqs[NSAMP];
uint16_t idx = 0;

/*---------
 * Miscelanea
 */
char hi[128];

/*---------
 * This is the actual sampling routine
 * The ADC conversions are stopped and the DMA channel acquisition started, it will be hung
 * until the first ADC result is placed on the ADC FIFO buffer which is drained to start
 * The DMA transfer is configured to take 8 bits samples ad as soon as a value is available placing
 * it on capture_buf 
 * Processing is then held until the DMA block is acquired 1000 samples at 5000 samples per second 
 * takes 200 mSecs which is a bit large for actual DSP, actual values need to be tuned later
 * result is left placed at capture_buf
 */
void sample(uint8_t *capture_buf) {

  adc_fifo_drain();
  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
			capture_buf,    // dst
			&adc_hw->fifo,  // src
			NSAMP,          // transfer count
			true            // start immediately
			);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
}

/*----------------
 * setup()
 * 
 */
void setup() {
  Serial.begin( 115200 );
  while(!Serial);
  Serial.setTimeout( 1000 );

/*---------------
 * initializes the ADC ports
 */
  adc_gpio_init(ADC_PIN+CAPTURE_CHANNEL);
  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 true     // Shift each sample to 8 bits when pushing to FIFO
		 );

/*------------------
 * Set the divider which defines the number of samples
 */

  adc_set_clkdiv(CLOCK_DIV);

/*------------------
 * Let things settle
 */
  sleep_ms(1000);

/*------------------  
 * Initialize now the DMA channel
 * Start by selecting an unused DMA channel and get the default configuration for it.
 * Then establish that samples will be 8 bytes and placed incrementally in the write buffer
 * Finally, sync the rate of transfer to the availability of ADC samples
 */
  dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8); 
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_ADC);

/*------------------
 * Calculate the frequency associated with each bin, they are constant for a given number of samples and 
 * sampling rate
 */
  float f_max = FSAMP;
  float f_res = f_max / NSAMP;
  for (int i = 0; i < NSAMP; i++) {freqs[i] = f_res*i;}
}

/*--------------------
 * loop()
 */
void loop() {

/*--------------------
 * Define the bin buffer large enough to contain the required number of samples
 * ini
 */
  uint8_t cap_buf[NSAMP];
/*  
  for (int j=0;j<NSAMP;j++) {
      cap_buf[j]=0;
  }
*/

/*---------------------
 * Perform actual sampling, take NSAMP samples at FSAMP sampling rate
 * place the result on the sampling buffer
 */
  sample(cap_buf);

/*---------------------
 * Process the fast Fourier transform on the time domain data contained at cap_buf
 */

 /*---------------------
  * define FFT related areas
  */
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP,false,0,0);
  
 /*---------------------
  * pre-process the time domain data, obtain the average energy level and take it as the DC 
  * component, so remove it.
  */
    // fill fourier transform input while subtracting DC component
    uint64_t sum = 0;
    for (int i=0;i<NSAMP;i++) {sum+=cap_buf[i];}
    float avg = (float)sum/NSAMP;
 
    for (int i=0;i<NSAMP;i++) {fft_in[i]=(float)cap_buf[i]-avg;}

/*----------------------------
 * Compute the actual FFT
 * Input bin information comes in fft_in (float) and frequency bin information comes in fft_out (float)
 */
    kiss_fftr(cfg , fft_in, fft_out);

/*---------------------------    
 * Compute power
 * 
 */
    float max_power = 0;
    int max_idx = 0;
/*---------------------------
 * Scan over frequency bins, any frequency above NSAMP/2 will be an alias (Nyquist theorem)
 * find the bucket with largest level of energy.
 * This is a test program which is supposed to be tested with discrete tones, this lacks
 * sense on a real spectrum full of signals.
 */
    for (int i = 1; i < NSAMP/2; i++) {
      float power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
      if (power>max_power) {
	       max_power=power;
	       max_idx = i;
      }
    }

    float max_freq = freqs[max_idx];

    sprintf(hi,"Average signal=%0.1f Peak frequency=%0.1f Hz\n",avg,max_freq);
    Serial.print(hi);

/*-----------------------------
 * freed resources
 */
    kiss_fft_free(cfg);  
}
