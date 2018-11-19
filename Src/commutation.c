/*
 * commutation.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */
#include "commutation.h"
#include "delay_uS.h"
#include "adc.h"
#include "foc_commutation.h"
//#define USE_COMPLEMENTARY_PWM

int32_t gl_rotorPos = 0;	//rotor position in degrees
int32_t gl_rotorInterval = 0;	//rotor TIME between 30 degrees of commutation (master converts to speed)
int gl_comm_step;	//rotor step index. used mainly for open->closed transitions

int high_phase = 0;
int low_phase = 0;

//correct for r2/r3
//TIM_HandleTypeDef * pwmHandle[6] = {&htim1, &htim3, &htim1, &htim3, &htim1, &htim3};
//uint32_t pwmChannel[6] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_3,TIM_CHANNEL_4};	//actual, correct mapping
//const comm_step fw[6] = {{INHA,INLB},{INHA,INLC},{INHB,INLC},{INHB,INLA},{INHC,INLA},{INHC,INLB}};

#ifdef USE_COMPLEMENTARY_PWM
//								high			low			neutral
const comm_step fw[6] = { 	{TIM_CHANNEL_1,TIM_CHANNEL_2, MASK_3},
		{TIM_CHANNEL_1,TIM_CHANNEL_3, MASK_2},
		{TIM_CHANNEL_2,TIM_CHANNEL_3, MASK_1},
		{TIM_CHANNEL_2,TIM_CHANNEL_1, MASK_3},
		{TIM_CHANNEL_3,TIM_CHANNEL_1, MASK_2},
		{TIM_CHANNEL_3,TIM_CHANNEL_2, MASK_1} };

//								high			low			neutral
const comm_step bw[6] = { 	{TIM_CHANNEL_2,TIM_CHANNEL_1, MASK_3},
		{TIM_CHANNEL_2,TIM_CHANNEL_3, MASK_1},
		{TIM_CHANNEL_1,TIM_CHANNEL_3, MASK_2},
		{TIM_CHANNEL_1,TIM_CHANNEL_2, MASK_3},
		{TIM_CHANNEL_3,TIM_CHANNEL_2, MASK_1},
		{TIM_CHANNEL_3,TIM_CHANNEL_1, MASK_2} };

#endif

#ifndef USE_COMPLEMENTARY_PWM
const comm_step bw[6] = { 	{TIM_CHANNEL_1,TIM_CHANNEL_2, MASK_3_S1},
		{TIM_CHANNEL_1,TIM_CHANNEL_3, MASK_2_S1},
		{TIM_CHANNEL_2,TIM_CHANNEL_3, MASK_1_S2},
		{TIM_CHANNEL_2,TIM_CHANNEL_1, MASK_3_S2},
		{TIM_CHANNEL_3,TIM_CHANNEL_1, MASK_2_S3},
		{TIM_CHANNEL_3,TIM_CHANNEL_2, MASK_1_S3} };

const comm_step fw[6] = { 	{TIM_CHANNEL_2,TIM_CHANNEL_1, MASK_3_S2},
		{TIM_CHANNEL_2,TIM_CHANNEL_3, MASK_1_S2},
		{TIM_CHANNEL_1,TIM_CHANNEL_3, MASK_2_S1},
		{TIM_CHANNEL_1,TIM_CHANNEL_2, MASK_3_S1},
		{TIM_CHANNEL_3,TIM_CHANNEL_2, MASK_1_S3},
		{TIM_CHANNEL_3,TIM_CHANNEL_1, MASK_2_S3} };
#endif

const int backwardADCBemfTable[6] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A};
const int backwardEdgePolarity[6] = {FALLING, RISING, FALLING, RISING, FALLING, RISING};

const int forwardADCBemfTable[6] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_B};
const int forwardEdgePolarity[6] = {FALLING, RISING, FALLING, RISING, FALLING, RISING};	//this might be wrong

int gl_zero_cross_point = 0;
const int dead_time_uS = 50;

void estSpeedPos(const comm_step * commTable, int phase_delay_uS)
{
	if(commTable == fw)
		gl_rotorPos++;
	else
		gl_rotorPos--;
	gl_rotorInterval = (phase_delay_uS>>1)+phase_delay_uS;		//approximate
}
/*
 * helper function for the comm_step type, which loads all unreferenced pwms as 0 and all referenced pwms as duty
 */
void load_pwm_step(comm_step c, int duty)
{
	/*
	 * TODO: figure out why zero crossing doesn't work when properly setting CCR4
	 */
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | c.mask;
	__HAL_TIM_SET_COMPARE(&htim1, c.H, duty);
	__HAL_TIM_SET_COMPARE(&htim1, c.L, 0);
}


/*
 * align partiular to the global commutation table
 */
void align(const comm_step * commTable)
{
	//	openLoop(fw,1500,5000);
	load_pwm_step(commTable[5],200);
	HAL_Delay(1);
}

/*
 *	gets the average of all 3 bemf channel vals
 */
int initZeroCrossPoint(uint16_t * adc_data_vals)
{
	TIM1->CCER = (TIM1->CCER | ENABLE_ALL);
	int i;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	HAL_Delay(1);
	int avg_val[3] = {0};
	for(i=0;i<100;i++)
	{
		avg_val[ADC_CHAN_BEMF_C-3]+=adc_data_vals[ADC_CHAN_BEMF_C];
		avg_val[ADC_CHAN_BEMF_B-3]+=adc_data_vals[ADC_CHAN_BEMF_B];
		avg_val[ADC_CHAN_BEMF_A-3]+=adc_data_vals[ADC_CHAN_BEMF_A];
		delay_T14_us(200);
	}
	for(i=0;i<3;i++)
		avg_val[i]/=100;
	int total_average = (avg_val[0]+avg_val[1]+avg_val[2])/3;
	return total_average/2;
}

//TODO: make this not active brake
void stop()
{
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	TIM1->CCER = (TIM1->CCER & DIS_ALL);
}

void brake()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

void hardBrake(int duty)
{
	if(duty > 500)
		duty = 500;
	else if(duty < 0)
		duty = 0;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
}

//void openLoopLin(const comm_step * commTable, int duty)
//{
//	int stepDelay = -17*duty+8000;
//	openLoop(commTable, duty, stepDelay);
//	//	openLoopSinusoidal(commTable,duty,stepDelay);
//
//}

void openLoopEst(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity, int duty, int phase_delay_uS)
{
	if(phase_delay_uS < 15)
		phase_delay_uS = 15;
	int stepIdx;
	for(stepIdx = 0; stepIdx < 6; stepIdx++)
	{
		load_pwm_step(commTable[stepIdx],duty);

		TIM14->CNT = 0;
		int timOver = 0;
		int uS=0;
		//		int zero_cross_thresh = 1000;
		while(!timOver)
		{
			uS = TIM14->CNT;
			if(uS>phase_delay_uS)
				timOver=1;
		}
	}

}


/*
 * TODO: test, and add load_pwm_step function
 * open loop commutation table sequencing
 */
void openLoop(const comm_step * commTable, int duty, int phase_delay_uS)
{
	if(phase_delay_uS < 15)
		phase_delay_uS = 15;
	int stepIdx;
	for(stepIdx = 0; stepIdx < 6; stepIdx++)
	{
		load_pwm_step(commTable[stepIdx],duty);
		delay_T14_us(phase_delay_uS);

		estSpeedPos(commTable, phase_delay_uS);
	}
}




/*
 * Open loop spinup for closed-loop transition
 * INPUTS:
 *
 */
void openLoopAccel(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity)
{
	int duty_max_thresh = 1000;
	int phase_delay_uS;
	int duty = 500;
	align(fw);
	int stepIdx = 0;
	for(phase_delay_uS = 12000; phase_delay_uS >= 2500; phase_delay_uS-=300)
	{
		//		float Fs = ((float)phase_delay_uS/1000000.0)*3*2*PI;
		//		float t = 0;
		//		TIM14->CNT = 0;
		//		while(TIM14->CNT < phase_delay_uS)
		//		{
		//			float drive = (float)duty*sin(Fs*t+deg_60);
		//			t+=.000001;
		//			load_pwm_step(commTable[stepIdx],drive);
		//		}
		load_pwm_step(commTable[stepIdx],duty);
		delay_T14_us(phase_delay_uS);

		stepIdx++;
		if(stepIdx >= 6)
			stepIdx = 0;
		gl_comm_step = stepIdx;
		//openLoopEst(commTable, bemfTable, edgePolarity, duty, phase_delay_uS);
		duty+=15;
		if(duty>duty_max_thresh)
			duty = duty_max_thresh;
		estSpeedPos(commTable, phase_delay_uS);
	}

}

uint32_t openloop_spinup_ts = 0;
uint32_t openloop_spinup_window_count = 0;
uint32_t stall_ts = 0;		//marks the	time which stall detection ends. if the current time is greater than this time, allow commutation
uint32_t stall_led_ts = 0;
/*
 * Closed loop sensorless bldc motor control.
 * INPUTS:  commTable: table dictating INH and INL pairs in sequence for 1 full electrical commutation
 * 			bemfTable: table containing the unfed phase of each commutation step
 * 			edgePolarity: table containing the polarity of the edge of the BEMF of the unfed phase (rising or falling edge)
 * 			duty: the PWM duty cycle determining torque/speed of the motor
 */
void closedLoop(const comm_step * commTable, const int * bemfTable,  const int * edgePolarity, int duty)
{
	int steps_taken = 0;
	if(HAL_GetTick() > stall_ts)
	{
		while(steps_taken < 6)
		{
			int stepIdx = gl_comm_step;
			int zero_cross_event = 0;
			load_pwm_step(commTable[stepIdx],duty);
			//		load_comm_step(commTable[stepIdx], duty, edgePolarity[stepIdx]);
			TIM14->CNT = 0;
			int uS_count = 0;

			while(zero_cross_event==0)
			{
				uS_count = TIM14->CNT;

				if(edgePolarity[stepIdx] == RISING)
				{
					if(dma_adc_trap[bemfTable[stepIdx]] >= gl_zero_cross_point)
					{
						zero_cross_event = 1;
						HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
					}
				}
				else if (edgePolarity[stepIdx] == FALLING)
				{
					if(dma_adc_trap[bemfTable[stepIdx]] <= gl_zero_cross_point)
					{
						zero_cross_event = 1;
						HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
					}
				}
				if(uS_count>=15000)	//if attemped for over 9000uS, accelerate and start over (failed closed loop commutation)
				{

					if(HAL_GetTick()-openloop_spinup_ts < 100)		//only upcount if you're spinning up in the same 'window' determined by timeframe
					{
						if(openloop_spinup_window_count > 15)
						{
							TIM1->CCER = (TIM1->CCER & 0xFAAA);
							stall_ts = HAL_GetTick()+1000;
							return;	//if you've tried over 15 times, trigger stall detection for some time (1 second?)
						}
						if(openloop_spinup_window_count > 4)
						{
							stall_ts = HAL_GetTick()+4;
							return;
						}
						openloop_spinup_window_count++;
					}
					openloop_spinup_ts = HAL_GetTick();
					zero_cross_event = 1;
					uS_count = 2;
					openLoopAccel(commTable, bemfTable, edgePolarity);
					return;
				}
			}

			/* you're halfway through the step, and need to track rotation counts for FOC.*/
			//		unwrap( theta_rel_rad(), &foc_theta_prev);
			//		mech_theta_prev = (float)gl_rotorPos*ONE_BY_THREE_PI;

			/*
			 *	keep switching but flip polarity
			 */
			//		delay_T14_us(uS_count/2);	//in theory, this should not be uS_count/2. however, this controller draws more current when the delay is that long.
			TIM14->CNT = 0;
			while(TIM14->CNT <= uS_count>>1);

			estSpeedPos(commTable, uS_count);	//update speed and position

			//		gl_zero_cross_point = gl_virtual_neutral/3;
			gl_comm_step++;
			if(gl_comm_step >= 6)
				gl_comm_step = 0;
			steps_taken++;
		}
	}
	else
	{
		TIM1->CCER = (TIM1->CCER & 0xFAAA);
		if(HAL_GetTick() > stall_led_ts)
		{
			stall_led_ts = HAL_GetTick() + 50;
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
		}
	}

}
