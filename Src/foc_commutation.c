/*
 * foc_commutation.c
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */
#include "foc_commutation.h"
#include "adc.h"
#include "commutation.h"
#include <math.h>

float mL = 1;	//should be 3/2*motor L
float mR = 1;
float Psi = 1;
float L;
float R;

#define ONE_BY_SQRT_3 	0.57735026919
#define TWO_BY_SQRT_3 	1.15470053838
#define SQRT_3_BY_2 	0.866025404

#define ADC_CURRENT_CONV_RATIO 0.002877371651785714285714

void init_observer()
{
	L = 3.0/2.0*mL;
	R = 3.0/2.0*mR;
}

/*
 * based on http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5200471
 */
unsigned long int t;
float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2)
{

	float dt = .0001;	//TODO: estimate dt

	float y1,y2;
	y1 = -R*i_a+v_a;
	y2 = -R*i_b+v_b;

	float gamma = 1;	//find later
	float L_i_a = L*i_a;
	float L_i_b = L*i_b;
	float n1,n2;
	n1 = *x1-L_i_a;
	n2 = *x2-L_i_b;
	float e = Psi*Psi - (n1*n1+n2*n2);
	float xdot1,xdot2;
	xdot1 = y1 + gamma*.5 + n1*e;
	xdot2 = y2 + gamma*.5 + n2*e;
	*x1 += xdot1*dt;
	*x2 += xdot2*dt;

	/*
	 * TODO: try putting the observer in a loop, as in vesc.
	 */

	return atan2((*x2-L_i_b),(*x1-L_i_a));

}

#define INHA 0
#define INLA 1
#define INHB 2
#define INLB 3
#define INHC 4
#define INLC 5

/*
 *	convert each phase adc voltage to actual current value
 */
void conv_raw_current(float * i_a, float * i_b, float * i_c)
{
	*i_a = (float)(dma_adc_raw[ADC_CHAN_CURRENT_A]-gl_current_input_offset)*ADC_CURRENT_CONV_RATIO;
	*i_b = (float)(dma_adc_raw[ADC_CHAN_CURRENT_B]-gl_current_input_offset)*ADC_CURRENT_CONV_RATIO;
	*i_c = (float)(dma_adc_raw[ADC_CHAN_CURRENT_C]-gl_current_input_offset)*ADC_CURRENT_CONV_RATIO;
	/*
	 * TODO: find a way to calculate the high phase based on what switches are active
	 */
	//	int dHA = __HAL_TIM_GET_COMPARE(pwmHandle[INHA],pwmChannel[INHA]);
	//	int dHB = __HAL_TIM_GET_COMPARE(pwmHandle[INHB],pwmChannel[INHB]);
	//	int dHC = __HAL_TIM_GET_COMPARE(pwmHandle[INHC],pwmChannel[INHC]);
	//	int dLA = __HAL_TIM_GET_COMPARE(pwmHandle[INLA],pwmChannel[INLA]);
	//	int dLB = __HAL_TIM_GET_COMPARE(pwmHandle[INLB],pwmChannel[INLB]);
	//	int dLC = __HAL_TIM_GET_COMPARE(pwmHandle[INLC],pwmChannel[INLC]);
	//	if(dHA > 0 && dLB > 0)
	//		*i_a -= *i_b;
	//	if(dHA > 0 && dLC > 0)
	//		*i_a -= *i_c;


}

#define SQRT_3 1.7320508075688772935274463415f

/*
 * procedure:
 * First divide (alpha,beta) vector into 4 quadrants.
 * Then, for each quadrant, check which sector. two possibilities for each.
 * Example check for sector 1:
 * is in sector one if: 0 < atan2(beta/alpha) < 60*PI/180
 * -> beta/alpha < tan(60*PI/180)
 * -> beta/alpha < sqrt(3)
 * -> beta/sqrt(3) < alpha
 *
 * once sectors are parsed, apply the vesc equations to convert to times.
 * note: every document i see has a slightly different version of these equations. i'm going
 * with vesc's since i don't understand it right now, but i know vesc works. hopefully so will this
 *
 */
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC)
{
	float pwm_half_period = (float)pwm_period_cnt;
	uint32_t sector;
	if(beta >= 0.0f)	//quadrant 1 or two
	{
		if(alpha >= 0.0f)	//quadrant 1
		{
			if(beta <= alpha*SQRT_3)	//sector 1
				sector = 1;
			else							//sector 2
				sector = 2;
		}
		else				//quadrant 2
		{
			if(beta <= alpha*-SQRT_3)	//sector 2
				sector = 3;
			else							//sector 3
				sector = 2;
		}
	}
	else			//quadrant 3 or 4
	{
		if(alpha >= 0.0f)	//quadrant 4
		{
			if(beta < -SQRT_3*alpha)
				sector = 5;
			else
				sector = 6;
		}
		else				//quadrant 3
		{
			if(beta < alpha*SQRT_3)
				sector = 5;
			else
				sector = 4;
		}
	}

	switch (sector) {
	case 1:
	{
		uint32_t t1 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t2 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t1 - t2) / 2;
		*tB = *tA + t1;
		*tC = *tB + t2;
		break;
	}
	case 2:
	{
		uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t2 - t3) / 2;
		*tA = *tB + t3;
		*tC = *tA + t2;
		break;
	}
	case 3:
	{
		uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t3 - t4) / 2;
		*tC = *tB + t3;
		*tA = *tC + t4;
		break;
	}
	case 4:
	{
		uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t4 - t5) / 2;
		*tB = *tC + t5;
		*tA = *tB + t4;

		break;
	}
	case 5:
	{
		uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t5 - t6) / 2;
		*tA = *tC + t5;
		*tB = *tA + t6;
		break;
	}
	case 6:
	{
		uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t6 - t1) / 2;
		*tC = *tA + t1;
		*tB = *tC + t6;
		break;
	}
	}


	return sector;
}

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta)
{
	(*i_alpha) = i_a;
	*i_beta = ONE_BY_SQRT_3*i_a + TWO_BY_SQRT_3*i_b;
}
void park_transform(float i_alpha, float i_beta, float theta, float * i_q, float * i_d)
{
	*i_d = i_alpha*cos(theta)+i_beta*sin(theta);
	*i_q = -i_alpha*sin(theta)+i_beta*cos(theta);
}
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c)
{
	*i_a = i_alpha;
	*i_b = -.5*i_alpha + SQRT_3_BY_2*i_beta;
	*i_c = -.5*i_alpha - SQRT_3_BY_2*i_beta;
}
void inverse_park_transform(float i_q, float i_d, float theta, float * i_alpha, float * i_beta)
{
	*i_alpha = i_d*cos(theta) - i_q*sin(theta);
	*i_beta = i_d*sin(theta) + i_q*cos(theta);
}
