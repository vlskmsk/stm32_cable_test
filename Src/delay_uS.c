/*
 * delay_uS.c
 *
 *  Created on: Nov 7, 2017
 *      Author: Ocanath
 */
#include "init.h"
#include "delay_uS.h"
#include "comm.h"

unsigned long int TIM14_ms_count = 0;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		TIM14_ms_count++;
	}
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

