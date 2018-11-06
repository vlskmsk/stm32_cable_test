/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"

//uint16_t dma_adc_raw[NUM_ADC] = {0};	//initialized, DMA access array
//int bemf_adc_map[3] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A};

//int zero_cross_flag[NUM_ADC] = {0};
//int below_thresh[3] = {0};
//int below_thresh_prev[3] = {0};

//int gl_bemf_adc_record_idx = 0;
//int gl_adc_capture_flag = 0;

//float iA_filt = 0;
//uint8_t shunt_state;

/*
 * TODO: implement single section IIR filter at a decimated adc frequency.
 * Most likely, 2kHz will be a sufficient cutoff frequency at a 5khz sampling frequency.
 * Experiment with filtering the raw adc values (requiring 3 filters) and
 * filtering the converted adc values (requiring only 2 filters but conversion in the
 * handler).
 *
 * Also, try decimation through changing the ADC timer base from the 14Mhz internal clock to a prescaled SYSCLK.
 * if the appropriate prescaler can be used, it will save some time in the handler.
 */
//uint16_t adc_I[3] = {0,0,0};
//uint16_t adc_V[3] = {0,0,0};

float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}

