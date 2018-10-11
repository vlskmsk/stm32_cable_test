/*
 * mag-encoder.c
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */
#include "mag-encoder.h"
#include <math.h>

//const int16_t cos_mid = 1992;
//const int16_t sin_mid = 1992;		//my encoder
int16_t cos_mid = 1910;
int16_t sin_mid = 1911;		//my encoder
//float cos_correct = 0.0002952465308532625;
//float sin_correct = 0.0002989536621823618;
//const int16_t sin_mid = 1250;		//steven encoder
//const int16_t cos_mid = 1243.5;

float align_offset = 0;				//offset angle IN RADIANS

//from 0 to pi/2
float atan2_approx(float sinVal, float cosVal)
{
	float abs_s = sinVal;
	if(abs_s < 0)
		abs_s = -abs_s;
	float abs_c = cosVal;
	if(abs_c < 0)
		abs_c = -abs_c;
	float min_v = abs_c;
	float max_v = abs_s;
	if(abs_s < abs_c)
	{
		min_v = abs_s;
		max_v = abs_c;
	}
	float a = min_v/max_v;
	float sv = a*a;
	float r = ((-0.0464964749 * sv + 0.15931422)*sv- 0.327622764) * sv * a + a;
	if(abs_s > abs_c)
		r = 1.57079637 -r;
	if(cosVal < 0)
		r = 3.14159274 - r;
	if(sinVal < 0)
		r = -r;
	return r;
}

float theta_abs_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
//	return atan2((float)sinVal,(float)cosVal);
	return atan2_approx((float)sinVal,(float)cosVal);
}

float theta_rel_rad()
{
	int16_t sinVal = (dma_adc_raw[ADC_SIN_CHAN]-sin_mid);
	int16_t cosVal = (dma_adc_raw[ADC_COS_CHAN]-cos_mid);
	//	return atan2((float)sinVal*sin_correct,(float)cosVal*cos_correct) - align_offset;
	return atan2_approx((float)sinVal,(float)cosVal) - align_offset;
}


//float theta_abs_rad()
//{
//	int16_t sinVal = (dma_adc_raw[ADC_SIN_CHAN]-sin_mid);
//	int16_t cosVal = (dma_adc_raw[ADC_COS_CHAN]-cos_mid);
//
//	return atan2((float)sinVal*sin_correct,(float)cosVal*cos_correct);
//}

/*
 * Fast taylor series based approximation of sin(theta).
 * TODO: implement lookup table implementation
 * NOTE:
 * designed to play with fast atan2, meaning it's only valid for positive quadrants I, II and negative quadrants III, IV.
 */
float sin_fast(float theta)
{
	uint8_t is_neg = 0;
	if(theta > HALF_PI && theta <= PI)	// if positive and in quadrant II, put in quadrant I (same)
	{
		theta = PI - theta;
	}
	else if (theta >= PI && theta < THREE_BY_TWO_PI)  // if positive and in quadrant III (possible for cosine)
	{
		is_neg = 1;
		theta = theta - PI;
	}

	else if (theta > THREE_BY_TWO_PI && theta < TWO_PI)  // if positive and in quadrant IV (edge case of cosine, rare but possible)
	{
		theta = theta - TWO_PI;
	}
	else if (theta < -HALF_PI && theta >= -PI ) // if negative and in quadrant III,
	{
		is_neg = 1;
		theta = PI + theta;
	}

	float theta_2 = theta*theta;
	float theta_3 = theta_2*theta;
	float theta_5 = theta_3*theta_2;
	float res = theta-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL;
	if(is_neg == 1)
		return -res;
	else
		return res;
}
float cos_fast(float theta)
{
	return sin_fast(theta + HALF_PI);
}

/*
	fast 2pi mod. needed for sin and cos FAST for angle limiting
 */
#define ONE_BY_TWO_PI 0.1591549
float fmod_2pi(float in)
{
	uint8_t aneg = 0;
	float in_eval = in;
	if(in < 0)
	{
		aneg = 1;
		in_eval = -in;
	}
	float fv = (float)((int)(in_eval*ONE_BY_TWO_PI));
	if(aneg == 1)
		fv = (-fv)-1;
	return in-TWO_PI*fv;
}

const float rad_to_deg = 180.0/M_PI;
float theta_abs_deg()
{
	return theta_abs_rad()*rad_to_deg;
}

float theta_rel_deg()
{
	return theta_rel_rad()*rad_to_deg;
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
		if(dma_adc_raw[ADC_SIN_CHAN] > s_max)
			s_max = dma_adc_raw[ADC_SIN_CHAN];
		if(dma_adc_raw[ADC_SIN_CHAN] < s_min)
			s_min = dma_adc_raw[ADC_SIN_CHAN];

		if(dma_adc_raw[ADC_COS_CHAN] > c_max)
			c_max = dma_adc_raw[ADC_COS_CHAN];
		if(dma_adc_raw[ADC_COS_CHAN] < c_min)
			c_min = dma_adc_raw[ADC_COS_CHAN];


		float theta = 371*sin_fast( (fmod_2pi(.5*time_seconds() + PI) - PI) );
		theta = fmod_2pi(theta + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		float i_alpha,i_beta;
		uint32_t tA,tB,tC;
		inverse_park_transform(.15, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
	}
	TIMER_UPDATE_DUTY(0,0,0);
	cos_mid = (s_max + s_min)/2;
	sin_mid = (c_max + c_min)/2;
}

float unwrap(float theta,float * prev_theta)
{
	float dif = fmod_2pi(fmod_2pi(*prev_theta) - theta + PI);
	if(dif < 0)
		dif = dif + TWO_PI;
	dif = dif - PI;
	float theta_ret = *prev_theta - dif;
	*prev_theta = theta_ret;
	return theta_ret;
}

