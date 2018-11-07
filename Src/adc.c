/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"


void adc_init(control_type mode)
{
	HAL_ADC_Stop_DMA(&hadc);
	ADC1->CHSELR = 0;
	ADC1->CR |= ADSTP;		//stop any conversions
	ADC1->CR |= ADDIS;		//disable

	if(mode == TRAPEZOIDAL_MODE)
	{
		MX_ADC_Init_TRAP();
		HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_trap, NUM_ADC_TRAP);
	}
	else if (mode == FOC_MODE)
	{
		MX_ADC_Init_FOC();
		HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_foc, NUM_ADC_FOC);
	}
}

float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}

