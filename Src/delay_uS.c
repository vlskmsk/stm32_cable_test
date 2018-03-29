/*
 * delay_uS.c
 *
 *  Created on: Nov 7, 2017
 *      Author: Ocanath
 */
#include "init.h"
#include "delay_uS.h"
#include "comm.h"

void delay_T14_us(int cycles)
{

	//TIM2->EGR |= 0b1;	//reinitialize the counter register
	//htim14.Instance->CNT = 0;
	//TIM14->CNT;
	TIM14->CNT = 0;
	while(TIM14->CNT <= cycles);
}

