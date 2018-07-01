/*
 * mag-encoder.c
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */
#include "mag-encoder.h"
#include <math.h>

//const int16_t cos_mid = 1992;
//const int16_t sin_mid = 1992;		//my encoder
const int16_t cos_mid = 1986;
const int16_t sin_mid = 2006;		//my encoder

//const int16_t sin_mid = 1250;		//steven encoder
//const int16_t cos_mid = 1243.5;

float align_offset = 0;				//offset angle IN RADIANS

float theta_abs_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
	return atan2((float)sinVal,(float)cosVal);
}

float theta_rel_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
	return atan2((float)sinVal,(float)cosVal) - align_offset;
}
const float rad_to_deg = 180.0/M_PI;
float theta_abs_deg()
{
	return theta_abs_rad()*rad_to_deg;
}

float theta_rel_deg()
{
	return theta_rel_rad()*rad_to_deg;
}

