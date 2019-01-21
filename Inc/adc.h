/*
 * adc.h
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#ifndef ADC_H_
#define ADC_H_
#include "init.h"

typedef enum {FOC_MODE, TRAPEZOIDAL_MODE, HYBRID_MODE} control_type;

#define ADEN 	0x00000001
#define ADDIS 	0x00000002
#define ADSTART	0x00000004
#define ADSTP 	0x00000010
#define ADCAL	0x80000000

#define NUM_ADC_FOC  5


#define RISING 1
#define FALLING 0

#define ADC_CHAN_BEMF_C 3
#define ADC_CHAN_BEMF_B 4
#define ADC_CHAN_BEMF_A 5

#define ADC_CHAN_CURRENT_A 0
#define ADC_CHAN_CURRENT_B 1
#define ADC_CHAN_CURRENT_C 2

uint16_t dma_adc_foc[NUM_ADC_FOC];

void adc_init(control_type mode);

int gl_zero_cross_point;
int rotor_pos;

float adc_to_V(int adc);

#endif /* ADC_H_ */
