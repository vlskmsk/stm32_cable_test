/*
 * adc.h
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#ifndef ADC_H_
#define ADC_H_
#include "init.h"
#include "circ.h"
#include "iirSOS.h"

iirSOS filt[3];
uint8_t shunt_state;


#define NUM_ADC 6

#define RISING 1
#define FALLING 0

#define ADC_CHAN_BEMF_C 3
#define ADC_CHAN_BEMF_B 4
#define ADC_CHAN_BEMF_A 5

#define ADC_CHAN_CURRENT_A 0
#define ADC_CHAN_CURRENT_B 1
#define ADC_CHAN_CURRENT_C 2

float bemf_filtered[NUM_ADC];
int bemf_adc_map[3];
uint16_t dma_adc_raw[NUM_ADC];

int zero_cross_flag[NUM_ADC];
int gl_zero_cross_point;
int rotor_pos;

float adc_to_V(int adc);
int zeroCrossDetect(circularBuffer buf, int zcp, int polarity);
float iA_filt;
float iB_filt;
void init_23kHz_filt_coef();

#endif /* ADC_H_ */
