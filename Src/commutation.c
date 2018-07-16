/*
 * commutation.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */
#include "commutation.h"
#include "delay_uS.h"
#include "adc.h"


int16_t gl_rotorPos = 0;	//rotor position in degrees
int16_t gl_rotorInterval = 0;	//rotor TIME between 30 degrees of commutation (master converts to speed)
int gl_comm_step;	//rotor step index. used mainly for open->closed transitions

int high_phase = 0;
int low_phase = 0;

//correct for r2/r3
const commStep forwardCommutationTable[6] = {{INHA,INLB},{INHA,INLC},{INHB,INLC},{INHB,INLA},{INHC,INLA},{INHC,INLB}};
const int forwardADCBemfTable[6] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A};
const int forwardEdgePolarity[6] = {FALLING, RISING, FALLING, RISING, FALLING, RISING};

const commStep backwardCommutationTable[6] = {{INHC,INLB},{INHC,INLA},{INHB,INLA},{INHB,INLC},{INHA,INLC},{INHA,INLB}};
const int backwardADCBemfTable[6] = {ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_A,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_C};
const int backwardEdgePolarity[6] = {FALLING, RISING, FALLING, RISING, FALLING, RISING};	//this might be wrong


const int CCpwmHandle[6] = {TIM_CHANNEL_1,TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_3};


int gl_zero_cross_point = 0;
const int dead_time_uS = 50;

void estSpeedPos(const commStep * commTable, int phase_delay_uS)
{
	if(commTable == forwardCommutationTable)
		gl_rotorPos++;
	else if (commTable == backwardCommutationTable)
		gl_rotorPos--;
	gl_rotorInterval = phase_delay_uS/2;		//approximate
}

void start_pwm()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
/*
 * helper function for the commStep type, which loads all unreferenced pwms as 0 and all referenced pwms as duty
 */
void loadPWMCommStep(commStep c, int duty)
{
	uint32_t tA,tB,tC;
	tA = 500; tB = 500; tC = 500;
	if(c.phaseH == INHA)
		tA += duty;
	else if (c.phaseH == INHB)
		tB += duty;
	else if (c.phaseH == INHC)
		tC += duty;

	if(c.phaseL == INLA)
		tA -= duty;
	else if (c.phaseL == INLB)
		tB -= duty;
	else if (c.phaseL == INLC)
		tC -= duty;

	TIMER_UPDATE_DUTY(tA,tB,tC);

//	stop_pwm();
//
//	__HAL_TIM_SET_COMPARE(&htim1,CCpwmHandle[c.phaseH],duty);
//	__HAL_TIM_SET_COMPARE(&htim1,CCpwmHandle[c.phaseL],duty);
//
//	HAL_TIM_PWM_Start(&htim1, CCpwmHandle[c.phaseH]);
//	HAL_TIMEx_PWMN_Start(&htim1, CCpwmHandle[c.phaseL]);
}

/*
 * align partiular to the global commutation table
 */
void align(const commStep * commTable)
{
	//	openLoop(forwardCommutationTable,1500,5000);
	loadPWMCommStep(commTable[5],200);
	HAL_Delay(1);
}

/*
 *	gets the average of all 3 bemf channel vals
 */
int initZeroCrossPoint(uint16_t * adc_data_vals)
{
	int i;
	start_pwm();
	TIMER_UPDATE_DUTY(1000,1000,1000);
	delay_T14_us(10);
	int avg_val[3] = {0};
	for(i=0;i<100;i++)
	{
		avg_val[ADC_CHAN_BEMF_C-3]+=adc_data_vals[ADC_CHAN_BEMF_C];
		avg_val[ADC_CHAN_BEMF_B-3]+=adc_data_vals[ADC_CHAN_BEMF_B];
		avg_val[ADC_CHAN_BEMF_A-3]+=adc_data_vals[ADC_CHAN_BEMF_A];
		delay_T14_us(10);
	}
	for(i=0;i<3;i++)
		avg_val[i]/=100;
	int total_average = (avg_val[0]+avg_val[1]+avg_val[2])/3;
	return total_average/2;
}

void stop_pwm()
{
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void brake()
{
	TIMER_UPDATE_DUTY(0,0,0);
}

void hardBrake(int duty)
{
}

void openLoopLin(const commStep * commTable, int duty)
{
	int stepDelay = -17*duty+8000;
	openLoop(commTable, duty, stepDelay);
//	openLoopSinusoidal(commTable,duty,stepDelay);

}

void openLoopEst(const commStep * commTable, const int * bemfTable,  const int * edgePolarity, int duty, int phase_delay_uS)
{
	if(phase_delay_uS < 15)
		phase_delay_uS = 15;
	int stepIdx;
	for(stepIdx = 0; stepIdx < 6; stepIdx++)
	{
		loadPWMCommStep(commTable[stepIdx],duty);

		TIM14->CNT = 0;
		int timOver = 0;
		int uS=0;
		//		int zero_cross_thresh = 1000;
		int i;
		while(!timOver)
		{
			uS = TIM14->CNT;
			if(uS>phase_delay_uS)
				timOver=1;

			//			rollingAverage_2(&adc_filtered_buf[bemfTable[stepIdx]], dma_adc_raw[bemfTable[stepIdx]]);
			for(i=0;i<3;i++)
				updateCircBuf(&adc_filtered_buf[bemf_adc_map[i]],dma_adc_raw[bemf_adc_map[i]]);

			//			int zero_cross_event = zeroCrossDetect(adc_filtered_buf[bemfTable[stepIdx]], zero_cross_thresh,edgePolarity[stepIdx]);
		}
	}

}


/*
 * TODO: test, and add loadPWMCommStep function
 * open loop commutation table sequencing
 */
void openLoop(const commStep * commTable, int duty, int phase_delay_uS)
{
	if(phase_delay_uS < 15)
		phase_delay_uS = 15;
	int stepIdx;
	for(stepIdx = 0; stepIdx < 6; stepIdx++)
	{
		loadPWMCommStep(commTable[stepIdx],duty);
		float t = time_microseconds();
		while(time_microseconds()-t < phase_delay_uS);

		estSpeedPos(commTable, phase_delay_uS);
	}
}


/*
 * first shit-tastic crack at open loop sinusoidal control
 */
float deg_60 = PI/3;
void openLoopSinusoidal(const commStep * commTable, float speed, int phase_delay_uS)
{
	if(phase_delay_uS < 15)
		phase_delay_uS = 15;
	int stepIdx;
	float Fs = ((float)phase_delay_uS/1000000.0)*3*2*PI;
	float t = 0;
	for(stepIdx = 0; stepIdx < 6; stepIdx++)
	{
		TIM14->CNT = 0;
		while(TIM14->CNT < phase_delay_uS)
		{
			float drive = speed*sin(Fs*t+deg_60);
			t+=.000001;
			loadPWMCommStep(commTable[stepIdx],drive);
		}
	}
}

/*
 * first crack using sinusoidaloidal
 */
void openLoopLinSinusoidal(const commStep * commTable, int duty)
{
	int stepDelay = -17*duty+8000;
	openLoopSinusoidal(commTable, duty, stepDelay);
}


int estimate_zero_cross_thresh()
{
	int ret = (adc_filtered_buf[ADC_CHAN_BEMF_A].buf[1]+adc_filtered_buf[ADC_CHAN_BEMF_B].buf[1]+adc_filtered_buf[ADC_CHAN_BEMF_C].buf[1])/3;
	int dev =gl_zero_cross_point - ret;
	if(dev < 10 && dev > -10)
		return ret;
	else
		return gl_zero_cross_point;
}


/*
 * Open loop spinup for closed-loop transition
 * INPUTS:
 *
 */
void openLoopAccel(const commStep * commTable, const int * bemfTable,  const int * edgePolarity)
{
	int duty_max_thresh = 1000;
	int phase_delay_uS;
	int duty = 1000;
	align(forwardCommutationTable);
	int stepIdx = 0;
	for(phase_delay_uS = 9600; phase_delay_uS >= 960; phase_delay_uS-=400)
	{
		float Fs = ((float)phase_delay_uS/1000000.0)*3*2*PI;
		float t = 0;
		TIM14->CNT = 0;
		while(TIM14->CNT < phase_delay_uS)
		{
			float drive = (float)duty*sin(Fs*t+deg_60);
			t+=.000001;
			loadPWMCommStep(commTable[stepIdx],drive);
		}

//		loadPWMCommStep(commTable[stepIdx],duty);
//		delay_T14_us(phase_delay_uS);

		stepIdx++;
		if(stepIdx >= 6)
			stepIdx = 0;
		gl_comm_step = stepIdx;
		//openLoopEst(commTable, bemfTable, edgePolarity, duty, phase_delay_uS);
		duty+=30;
		if(duty>duty_max_thresh)
			duty = duty_max_thresh;
		estSpeedPos(commTable, phase_delay_uS);
	}
}

/*
 * Closed loop sensorless bldc motor control.
 * INPUTS:  commTable: table dictating INH and INL pairs in sequence for 1 full electrical commutation
 * 			bemfTable: table containing the unfed phase of each commutation step
 * 			edgePolarity: table containing the polarity of the edge of the BEMF of the unfed phase (rising or falling edge)
 * 			duty: the PWM duty cycle determining torque/speed of the motor
 */
void closedLoop(const commStep * commTable, const int * bemfTable,  const int * edgePolarity, int duty)
{
	int steps_taken = 0;
	while(steps_taken < 6)
	{
		int stepIdx = gl_comm_step;
		int zero_cross_event = 0;
		loadPWMCommStep(commTable[stepIdx],duty);
		TIM14->CNT = 0;
		int uS_count = 0;

		while(zero_cross_event==0)
		{
			uS_count = TIM14->CNT;
//			if(edgePolarity[stepIdx] == RISING && dma_adc_raw[bemfTable[stepIdx]] >= gl_zero_cross_point)
//				break;
//			else if(edgePolarity[stepIdx] == FALLING && dma_adc_raw[bemfTable[stepIdx]] <= gl_zero_cross_point)
//				break;
			zero_cross_event = ((edgePolarity[stepIdx] == RISING && dma_adc_raw[bemfTable[stepIdx]] >= gl_zero_cross_point) || (edgePolarity[stepIdx] == FALLING && dma_adc_raw[bemfTable[stepIdx]] <= gl_zero_cross_point));

			if(uS_count>=30000)	//if attemped for over 9000uS, accelerate and start over (failed closed loop commutation)
			{
				zero_cross_event = 1;
				uS_count = 2;
//				openLoopAccel(commTable, bemfTable, edgePolarity);
				return;
			}
		}
		delay_T14_us(uS_count/2);	//in theory, this should not be uS_count/2. however, this controller draws more current when the delay is that long.
		estSpeedPos(commTable, uS_count);	//update speed and position

		gl_comm_step++;
		if(gl_comm_step >= 6)
			gl_comm_step = 0;
		steps_taken++;
	}
}
