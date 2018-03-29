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

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//
//	//check for zero cross event
////	int i;
////	for(i=0;i<3;i++)
////	{
////		updateCircBuf(&gl_adcBuffer[bemf_adc_map[i]],dma_adc_raw[bemf_adc_map[i]]);
//////		rollingAverage_2(&gl_adcBuffer[bemf_adc_map[i]],dma_adc_raw[bemf_adc_map[i]]);
////	}
//}

//int zeroCrossDetect(circularBuffer buf, int zcp, int polarity)
//{
//	int middleIdx = (CIRC_BUF_SIZE-1)/2;
//	int i;
//	int low_flag = 0;
//	int high_flag = 0;
//
//	for(i=0;i<middleIdx;i++)
//	{
//		if(buf.buf[i] > zcp)
//			low_flag = 1;
//	}
//	for(i=middleIdx;i<CIRC_BUF_SIZE;i++)
//	{
//		if(buf.buf[i] <= zcp)
//			high_flag = 1;
//	}
//	int cross_event = 0;
//	if(high_flag == 1 && low_flag == 1)
//		cross_event = 1;
//	return cross_event;
//}

int zeroCrossDetect(circularBuffer buf, int zcp, int polarity)
{
	int middleIdx = (CIRC_BUF_SIZE-1)/2;
	int i;
	int low_flag = 1;
	int high_flag = 1;

	if(polarity == FALLING)
	{
		for(i=0;i<middleIdx;i++)
		{
			if(!(buf.buf[i] < zcp))
				low_flag = 0;
		}
		for(i=middleIdx;i<CIRC_BUF_SIZE;i++)
		{
			if(!(buf.buf[i] >= zcp))
				high_flag = 0;
		}
		int cross_event = 0;
		if(high_flag == 1 && low_flag == 1)
			cross_event = 1;
		return cross_event;
	}
	else if (polarity == RISING)
	{
		for(i=0;i<=middleIdx;i++)
		{
			if(!(buf.buf[i] > zcp))
				low_flag = 0;
		}
		for(i=middleIdx+1;i<CIRC_BUF_SIZE;i++)
		{
			if(!(buf.buf[i] <= zcp))
				high_flag = 0;
		}
		int cross_event = 0;
		if(high_flag == 1 && low_flag == 1)
			cross_event = 1;
		return cross_event;
	}
	else
		return 0;
}

float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}
