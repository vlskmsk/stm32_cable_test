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
const int16_t cos_mid = 2019;
const int16_t sin_mid = 1931;		//my encoder
float cos_correct = 0.0002952465308532625;
float sin_correct = 0.0002989536621823618;
//const int16_t sin_mid = 1250;		//steven encoder
//const int16_t cos_mid = 1243.5;

float align_offset = 0;				//offset angle IN RADIANS

//from 0 to pi/2

float theta_abs_rad()
{
	int16_t sinVal = (dma_adc_raw[ADC_SIN_CHAN]-sin_mid);
	int16_t cosVal = (dma_adc_raw[ADC_COS_CHAN]-cos_mid);

	return atan2((float)sinVal*sin_correct,(float)cosVal*cos_correct);
}

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

float theta_rel_rad()
{
	int16_t sinVal = (dma_adc_raw[ADC_SIN_CHAN]-sin_mid);
	int16_t cosVal = (dma_adc_raw[ADC_COS_CHAN]-cos_mid);
	//	return atan2((float)sinVal*sin_correct,(float)cosVal*cos_correct) - align_offset;
	return atan2_approx((float)sinVal*sin_correct,(float)cosVal*cos_correct) - align_offset;
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

/*
 * returns 0-1 sin and cos. i do not know of a way to offset, other than atan2->add offset -> sin cos
 */
void sin_cos_abs(float * sin_val, float * cos_val)
{
	*sin_val = (float)(sin_correct*(dma_adc_raw[ADC_SIN_CHAN]-sin_mid));
	*cos_val = (float)(cos_correct*(dma_adc_raw[ADC_COS_CHAN]-cos_mid));
}

