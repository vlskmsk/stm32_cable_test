/*
 * delay_uS.c
 *
 *  Created on: Nov 7, 2017
 *      Author: Ocanath
 */
#include "init.h"
#include "delay_uS.h"
#include "comm.h"

const long int ms_inc = (PSC_GEN_TIMER+1);

const float seconds_per_tick = (PSC_GEN_TIMER+1)/48000000.0;
const float milliseconds_per_tick = (PSC_GEN_TIMER+1)/48000.0;
const float microseconds_per_tick = (PSC_GEN_TIMER+1)/48.0;

unsigned long int TIM14_ms_count = 0;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		TIM14_ms_count+= ms_inc;	//see init.c
	}
}
float time_seconds()
{
	return (float)((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT))*seconds_per_tick;
}
float time_milliseconds()
{
	return (float)((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT))*milliseconds_per_tick;
}
float time_microseconds()
{
	return (float)((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT))*microseconds_per_tick;
}

unsigned long int TIM14_ms()
{
	return TIM14_ms_count;
}
void delay_T14_us(int cycles)
{

	//TIM2->EGR |= 0b1;	//reinitialize the counter register
	//htim14.Instance->CNT = 0;
	//TIM14->CNT;
	TIM14->CNT = 0;
	while(TIM14->CNT <= cycles);
}

