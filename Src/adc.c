/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"

#define BUFFER_SIZE 10

uint16_t dma_adc_raw[NUM_ADC] = {0};	//initialized, DMA access array
int bemf_adc_map[3] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A};

int zero_cross_flag[NUM_ADC] = {0};
int below_thresh[3] = {0};
int below_thresh_prev[3] = {0};

int gl_bemf_adc_record_idx = 0;
int gl_adc_capture_flag = 0;

float iA_filt = 0;
uint8_t shunt_state;

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	shunt_state = ((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10) << 2) | (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) << 1) | HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8));


	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
}

float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}

void init_23kHz_filt_coef()
{
	filt[0].b0 = 1;
	filt[0].b1 = 2;
	filt[0].b2 = 1;
	filt[0].a1 = -0.250697967679985;
	filt[0].a2 = 0.182705104359658;
	filt[0].gain = 0.233001784169918;


//
//	filt[1].a1 = 0.234667672575741;
//	filt[1].a2 = 0.144597823632472;
//	filt[1].b0 = 1;
//	filt[1].b1 = 1.73114829153189;
//	filt[1].b2 = 1;
//	filt[1].gain = 0.369662470756940;
//
//	filt[2].a1 = 0.132122696010434;
//	filt[2].a2 = 0;
//	filt[2].b0 = 1;
//	filt[0].b1 = 1;
//	filt[2].b2 = 0;
//	filt[2].gain = 0.566061348005217;
}
