/*
 * delay_uS.h
 *
 *  Created on: Nov 7, 2017
 *      Author: Ocanath
 */

#ifndef DELAY_US_H_
#define DELAY_US_H_



#define CONST_MS_TO_TICK 6000	//Formula: (SYSCLK/TIM14->Prescaler)/1000
const float seconds_per_tick;

void delay_T14_us(int cycles);
unsigned long int TIM14_ms();

float time_seconds();
float time_milliseconds();
float time_microseconds();


#endif /* DELAY_US_H_ */
