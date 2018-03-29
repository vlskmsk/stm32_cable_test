/*
 * commutation.h
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#ifndef COMMUTATION_H_
#define COMMUTATION_H_
#include "init.h"
#include "adc.h"
#include "circ.h"
#include <math.h>

#define PI 3.14159265359

#define MIN_CLOSED_DUTY 300
#define MIN_ACTIVE_DUTY 100
#define MIN_BRAKE_DUTY 10


#define NUM_PWM 6

#define INHA 0
#define INLA 1
#define INHB 2
#define INLB 3
#define INHC 4
#define INLC 5

circularBuffer adc_raw_buf[NUM_ADC];
circularBuffer adc_filtered_buf[NUM_ADC];

int16_t gl_rotorPos;	//rotor position in degrees
int16_t gl_rotorInterval;	//rotor TIME between 30 degrees of commutation (master converts to speed)

int high_phase;
int low_phase;

typedef struct commStep
{
	int phaseH;
	int phaseL;
}commStep;

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

/*
 * A->B
 * A->C
 * B->C
 * B->A
 * C->A
 * C->B
 */
const commStep forwardCommutationTable[6];
const int forwardADCBemfTable[6];
const int forwardEdgePolarity[6];

const commStep backwardCommutationTable[6];
const int backwardADCBemfTable[6];
const int backwardEdgePolarity[6];

void loadPWMCommStep(commStep c, int duty);
void align(const commStep * commTable);

void openLoopLin(const commStep * commTable, int duty);
void openLoop(const commStep * commTable, int duty, int phase_delay_uS);
void openLoopEst(const commStep * commTable, const int * bemfTable,  const int * edgePolarity, int duty, int phase_delay_uS);
void openLoopPrint(const commStep * commTable, int duty, int phase_delay_uS);
void openLoopAccel(const commStep * commTable, const int * bemfTable,  const int * edgePolarity);
void closedLoop(const commStep * commTable, const int * bemfTable,  const int * edgePolarity, int duty);
void start_pwm();
void stop_pwm();
void brake();
void hardBrake(int duty);

void openLoopLinSinusoidal(const commStep * commTable, int duty);
void openLoopSinusoidal(const commStep * commTable, float speed, int phase_delay_uS);
void closedLoopSinusoidal(const commStep * commTable, const int * bemfTable,  const int * edgePolarity, float speed);


void openloop_6step(int duty, int phase_delay_uS);
int initZeroCrossPoint(uint16_t * adc_data_vals);

#endif /* COMMUTATION_H_ */
