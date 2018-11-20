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
//#include <math.h>

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

#define DIS_ALL 0xFAAA		//and mask
#define MASK_1 0x0550		//or mask
#define MASK_2 0x0505		//or mask
#define MASK_3 0x0055		//or mask
#define ENABLE_ALL 0x0555	//or mask

#define MASK_1_S2 0x0510
#define MASK_1_S3 0x0150

#define MASK_2_S1 0x0501
#define MASK_2_S3 0x0105

#define MASK_3_S1 0x0051
#define MASK_3_S2 0x0015



int32_t gl_rotorPos;	//rotor position in degrees
int32_t gl_rotorInterval;	//rotor TIME between 30 degrees of commutation (master converts to speed)

int gl_comm_step;	//rotor step index. used mainly for open->closed transitions


typedef struct comm_step
{
	uint32_t H;
	uint32_t L;
	uint16_t mask;
}comm_step;

/*
 * A->B
 * A->C
 * B->C
 * B->A
 * C->A
 * C->B
 */
const comm_step fw[6];
const int forwardADCBemfTable[6];
const int forwardEdgePolarity[6];

const comm_step bw[6];
const int backwardADCBemfTable[6];
const int backwardEdgePolarity[6];

void load_pwm_step(comm_step c, int duty);
void align(const comm_step * commTable);

void openLoopLin(const comm_step * commTable, int duty);
void openLoop(const comm_step * commTable, int duty, int phase_delay_uS);
void openLoopEst(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity, int duty, int phase_delay_uS);
void openLoopPrint(const comm_step * commTable, int duty, int phase_delay_uS);
void openLoopAccel(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity);
void closedLoop(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity, int duty);
void stop();
void start_pwm();

void brake();
void hardBrake(int duty);

void openloop_6step(int duty, int phase_delay_uS);
int initZeroCrossPoint(uint16_t * adc_data_vals);

#endif /* COMMUTATION_H_ */
