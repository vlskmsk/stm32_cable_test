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
#include "delay_uS.h"
#include "mag-encoder.h"
#include "sin_lookup.h"

//#include <core_cm0.h>
float mL = 0;
float mR = 0;
float V_psi = 0;
float L;
float R;


const float adc_conv_A = 0.005754743303571;
const float adc_conv_B = 0.005754743303571;
const float adc_conv_C = 0.005754743303571;

int gl_sector=1;	//for SVM


/*
 * takes in iq,id reference currents
 * returns the current unwrapped motor position
 */
float i_a,i_b,i_c;
float i_alpha,i_beta;
float x_iq_PI = 0;	//torque pi state
float x_id_PI = 0;	//flux pi state
float uq = 0;
float ud = 0;
uint32_t tA,tB,tC;
float prev_theta = 0;
float theta_enc_prev=0;
void foc(float iq_ref,float id_ref)
{
	conv_raw_current(&i_a,&i_b, &i_c);
	clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

	gl_rotorInterval = TIM14->CNT;
	TIM14->CNT = 0;
	float theta_enc = unwrap( theta_rel_rad(), &foc_theta_prev);
	float theta_elec = theta_enc*elec_conv_ratio;
	theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
	float sin_theta = sin_lookup(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
	float cos_theta = cos_lookup(theta_elec);				//and the cosine for park and inverse park domains

	float i_q, i_d;
	park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q, &i_d);

	controller_PI(iq_ref, i_q, 0.01, 0.000000001, &x_iq_PI, &uq);		//this sort of works
	controller_PI(id_ref, i_d, 0.01, 0.000000000, &x_id_PI, &ud);		//high current

	inverse_park_transform(uq, ud, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?

	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tB,tA,tC);
}

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

	switch (sector)
	{
	case 1:
	{
		uint32_t t1 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t2 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t1 - t2) * .5;
		*tB = *tA + t1;
		*tC = *tB + t2;

		break;
	}
	case 2:
	{
		uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t2 - t3) * .5;
		*tA = *tB + t3;
		*tC = *tA + t2;
		break;
	}
	case 3:
	{
		uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t3 - t4) * .5;
		*tC = *tB + t3;
		*tA = *tC + t4;
		break;
	}
	case 4:
	{
		uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t4 - t5) * .5;
		*tB = *tC + t5;
		*tA = *tB + t4;

		break;
	}
	case 5:
	{
		uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t5 - t6) * .5;
		*tA = *tC + t5;
		*tB = *tA + t6;
		break;
	}
	case 6:
	{
		uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t6 - t1) * .5;
		*tC = *tA + t1;
		*tB = *tC + t6;
		break;
	}
	}
	gl_sector = sector;
	return sector;
}

/*
 * NOTE:  REQUIRES A->1, B->2, C->3 MAPPING!!!!!!!
 *
 * procedure:
 * First divide (alpha,beta) vector into 4 quadrants.
 * Then, for each quadrant, check which sector. two possibilities for each.
 * once sectors are parsed, convert to times.
 */
int svm_sinusoidal(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC)
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

	uint16_t mask;
	switch (sector)
	{
	case 1:
	{
		uint32_t t1 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t2 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t1 - t2) * .5;
		*tB = *tA + t1;
		*tC = *tB + t2;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_2;
		mask = MASK_2;
		break;
	}
	case 2:
	{
		uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t2 - t3) * .5;
		*tA = *tB + t3;
		*tC = *tA + t2;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_1;
		mask = MASK_1;
		break;
	}
	case 3:
	{
		uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t3 - t4) * .5;
		*tC = *tB + t3;
		*tA = *tC + t4;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_3;
		mask = MASK_3;
		break;
	}
	case 4:
	{
		uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t4 - t5) * .5;
		*tB = *tC + t5;
		*tA = *tB + t4;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_2;
		mask = MASK_2;
		break;
	}
	case 5:
	{
		uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t5 - t6) * .5;
		*tA = *tC + t5;
		*tB = *tA + t6;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_1;
		mask = MASK_1;
		break;
	}
	case 6:
	{
		uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t6 - t1) * .5;
		*tC = *tA + t1;
		*tB = *tC + t6;
		//		TIM1->CCER = (TIM1->CCER & DIS_ALL) | MASK_3;
		mask = MASK_3;
		break;
	}
	}
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | mask;
	switch(mask)
	{
	case MASK_1:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *tB);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *tC);
		break;
	case MASK_2:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *tA);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *tC);
		break;
	case MASK_3:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *tA);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *tB);
		break;
	};

	gl_sector = sector;
	return sector;
}

const float Kcor = 0.0;
const float u_max = .3;
const float u_min = -.3;
/*
 * x is state variable for integral delay
 */
void controller_PI(float i_q_ref, float i_q, float Kp, float Ki, float * x, float * u)
{
	float err = i_q_ref-i_q;
	*u = *x + Kp*err;
	*x = *x + Ki*err;
}

///*
// *Should have theoretically improved performance because currents are non-ideal and do not always sum to zero
// *(i.e. use all 3 instead of only 2 shunts...)
// */
//#define TWO_BY_THREE 0.666666667
//#define ONE_BY_THREE 0.333333333
//void clarke_transform_full(float i_a, i_b, i_c, float * i_alpha, float * i_beta)
//{
//	float ib_m_ic = (i_b-i_c);
//	(*i_alpha) = TWO_BY_THREE*i_a - ONE_BY_THREE*ib_m_ic;
//	(*i_beta) = TWO_BY_SQRT_3*ib_m_ic;
//}

/*
 *
 */
void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta)
{
	(*i_alpha) = i_a;
	*i_beta = ONE_BY_SQRT_3*i_a + TWO_BY_SQRT_3*i_b;
}
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d)
{
	*i_d = i_alpha*cos_theta+i_beta*sin_theta;
	*i_q = -i_alpha*sin_theta+i_beta*cos_theta;
}

void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c)
{
	*i_a = i_alpha;
	*i_b = -.5*i_alpha + SQRT_3_BY_2*i_beta;
	*i_c = -.5*i_alpha - SQRT_3_BY_2*i_beta;
}

void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta)
{
	*i_alpha = i_d*cos_theta - i_q*sin_theta;
	*i_beta = i_d*sin_theta + i_q*cos_theta;
}

void obtain_encoder_offset()
{
	float i_alpha,i_beta;
	inverse_park_transform(0, 0.5, 0, 1, &i_alpha, &i_beta);	//maybe call theta rel again?
	uint32_t tA,tB,tC;
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);		//TODO: since this produces (.2, -.1, -.1) -> (600, 450, 450), test (600, 400+50*sin(t), 400+50*sin(t)) and see if there
	// is any preturbation in the angle
	HAL_Delay(100);					//TODO: test if the angle is different depending on this value
	float avg_offset = 0;
	int i;
	int num_samples = 400;
	for(i=0;i<num_samples;i++)
	{
		avg_offset += theta_abs_rad();
		HAL_Delay(1);
	}
	align_offset = avg_offset/(float)num_samples;
}

float time_seconds()
{
	return ((float)HAL_GetTick())*.001;
}
void obtain_encoder_midpoints()
{
	float t_start = time_seconds();
	uint16_t s_max = 0;
	uint16_t s_min = 0xFFFF;
	uint16_t c_max = 0;
	uint16_t c_min = 0xFFFF;
	while(time_seconds() - t_start < 15)
	{
		if(dma_adc_foc[ADC_SIN_CHAN] > s_max)
			s_max = dma_adc_foc[ADC_SIN_CHAN];
		if(dma_adc_foc[ADC_SIN_CHAN] < s_min)
			s_min = dma_adc_foc[ADC_SIN_CHAN];

		if(dma_adc_foc[ADC_COS_CHAN] > c_max)
			c_max = dma_adc_foc[ADC_COS_CHAN];
		if(dma_adc_foc[ADC_COS_CHAN] < c_min)
			c_min = dma_adc_foc[ADC_COS_CHAN];


		float theta = 371*sin_fast( (fmod_2pi(.5*time_seconds() + PI) - PI) );
		theta = fmod_2pi(theta + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		float i_alpha,i_beta;
		uint32_t tA,tB,tC;
		inverse_park_transform(.15, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
//		TIMER_UPDATE_DUTY(tA,tB,tC);
	}
	TIMER_UPDATE_DUTY(0,0,0);
	sin_mid = (s_max + s_min)/2;
	cos_mid = (c_max + c_min)/2;
}

void get_current_cal_offsets()
{
	TIMER_UPDATE_DUTY(0,0,0);
	gl_current_input_offset_A = 0;
	gl_current_input_offset_B = 0;
	gl_current_input_offset_C = 0;
	int i;
	const int numSamples = 500;
	for(i=0;i<numSamples;i++)
	{
		gl_current_input_offset_A += dma_adc_foc[ADC_CHAN_CURRENT_A];
		gl_current_input_offset_B += dma_adc_foc[ADC_CHAN_CURRENT_B];
		gl_current_input_offset_C += dma_adc_foc[ADC_CHAN_CURRENT_C];
		delay_T14_us(10);
//		HAL_Delay(1);
	}
	gl_current_input_offset_A /= numSamples;
	gl_current_input_offset_B /= numSamples;
	gl_current_input_offset_C /= numSamples;
}

/*
 *	convert each phase adc voltage to actual current value
 */
void conv_raw_current(float * i_a, float * i_b, float * i_c)
{
	*i_a = (float)(gl_current_input_offset_A - dma_adc_foc[ADC_CHAN_CURRENT_A])*adc_conv_A;
	*i_b = (float)(gl_current_input_offset_B - dma_adc_foc[ADC_CHAN_CURRENT_B])*adc_conv_B;
	*i_c = (float)(gl_current_input_offset_C - dma_adc_foc[ADC_CHAN_CURRENT_C])*adc_conv_C;
}

/*
 * returns phase voltage based on svm output
 * TODO: scale outputs to bus voltage
 */
void convert_phase_voltage(float * va, float * vb, float * vc)
{
	int16_t t1 = TIM1->CCR1;
	int16_t t2 = TIM1->CCR2;
	int16_t t3 = TIM1->CCR3;
	*va = (float)t1*0.0160;	//period
	*vb = (float)t2*0.0160;	//period
	*vc = (float)t3*0.0160;	//period
}
//
///*
// * Seems to work...?
// * TODO: fix+test this
// */
//float est_R()
//{
//	TIMER_UPDATE_DUTY(0,0,0);
//	TIMER_UPDATE_DUTY(1000,1000,0);
//	HAL_Delay(10);
//	mR = 0;
//	int i;
//	for(i=0;i<10;i++)
//	{
//		float Va = (float)dma_adc_raw[ADC_CHAN_BEMF_A] * 0.000805664062 * 3.24725565716;
//		float Vb = (float)dma_adc_raw[ADC_CHAN_BEMF_B] * 0.000805664062 * 3.24725565716;
//		float i_a,i_b,i_c;
//		conv_raw_current(&i_a,&i_b, &i_c);
//		mR += (((Va+Vb)*.5)/i_c)/1.5;
//	}
//	mR = mR/(float)i;
//	TIMER_UPDATE_DUTY(0,0,0);
//	R = mR;
//	return R;
//}
//
//
///*
// * TODO: FIX THIS!!! general bones of getting L from time constant, but it does not work for the vishan. Tested with the
// * hobby, and didn't work either. should try shunt+ scope and measure the T that way.
// */
//float gl_I;
//float gl_T;
//float gl_V = 7.4;
//
//float est_L()
//{
//	float coil_r = R;
//
//	TIMER_UPDATE_DUTY(0,0,0);
//	HAL_Delay(100);
//	TIMER_UPDATE_DUTY(1000,1000,1000);
//	HAL_Delay(1);
//	float Va_m = (float)dma_adc_raw[ADC_CHAN_BEMF_A] * 0.000805664062 * 3.24725565716;
//	float Vb_m = (float)dma_adc_raw[ADC_CHAN_BEMF_B] * 0.000805664062 * 3.24725565716;
//	float Vc_m = (float)dma_adc_raw[ADC_CHAN_BEMF_C] * 0.000805664062 * 3.24725565716;
//	gl_V = (Va_m+Vb_m+Vc_m)/3.0;
//	float thresh = (gl_V/(2*coil_r))*.37;
//
//	while(1)
//	{
//		TIMER_UPDATE_DUTY(1000,1000,0);
//		uint32_t t_init = TIM14_ms()*1000+TIM14->CNT;
//		TIM14->CNT = 0;
//		while(1)	//might be the case that uS is not enough reso. for this kind of test
//		{
//			float i_a,i_b,i_c;
//			conv_raw_current(&i_a,&i_b, &i_c);
//			if(i_c > thresh)
//			{
//				gl_I = i_c;
//				gl_T = (float)((TIM14_ms()*1000+TIM14->CNT)-t_init);
//				break;
//			}
//			if(TIM14->CNT > 600)
//				break;
//		}
//		TIMER_UPDATE_DUTY(0,0,0);
//		printf("T found");
//		while(1);
//		//		float max_cur = coil_r
//	}
//	return 0;
//}


///*
// * TODO: write this properly. init l,r,vphz
// */
//void init_observer()
//{
//	//for vishan:
//	//Resistance of a single coil to CT = 0.86
//	//Inductance (measured from scope) of a SINGLE COIL to CT = 1.462
//	//VpHz = .01502703????????? (unknown) (was not able to measure via vesc)
//	//	est_R();
//	R = .86;
//	L = .0000405;
//	//	V_psi = 0.00152788745;	//in V/(rad/s), from vishan datasheet. close to vesc measurement...
//	V_psi = 0.00088212623;
//}
//
///*
// * based on http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5200471
// */
//float gl_t_prev = 0;
//float gl_t = 0;
//#define FOC_NUM_LOOPS 4
//const float inv_num_iter = 1/FOC_NUM_LOOPS;
//
//float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2)
//{
//	gl_t_prev = gl_t;
//	gl_t = time_seconds();
//	float dt = gl_t-gl_t_prev;
//
//	float ogain = 100000000*.5;	//gamma
//	float L_i_a = L*i_a;
//	float L_i_b = L*i_b;
//
//	float psi_sq = V_psi*V_psi;
//
//	float y1,y2;
//	y1 = -R*i_a+v_a;
//	y2 = -R*i_b+v_b;
//
//	/*
//	 * TODO: loop this calculation n times over measured dt for improved performance
//	 */
//	///////////////////////////////////////////////
//	float dt_step = dt*inv_num_iter;
//	int i;
//	for(i = 0; i < FOC_NUM_LOOPS; i++)
//	{
//		float n1,n2;
//		n1 = *x1-L_i_a;
//		n2 = *x2-L_i_b;
//
//		float e = psi_sq - (n1*n1+n2*n2);
//		float xdot1,xdot2;
//		xdot1 = y1 + ogain*n1*e;
//		xdot2 = y2 + ogain*n2*e;
//		*x1 += xdot1*dt_step;
//		*x2 += xdot2*dt_step;
//	}
//	///////////////////////////////////////////////
//
//	return atan2_approx( (*x2-L_i_b), (*x1-L_i_a));	//TODO: use fast atan2
//
//}

