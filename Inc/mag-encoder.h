/*
 * mag-encoder.h
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */

#ifndef MAG_ENCODER_H_
#define MAG_ENCODER_H_
#include "adc.h"
#define ADC_SIN_CHAN 	4
#define ADC_COS_CHAN 	3

#define ONE_BY_THREE_FACTORIAL 	0.16666666666f
#define ONE_BY_FIVE_FACTORIAL 	0.00833333333f
#define HALF_PI 				1.57079632679f
#define PI						3.14159265359f
#define THREE_BY_TWO_PI     	4.71238898038f
#define TWO_PI              	6.28318530718f
#define ONE_BY_TWO_PI 			0.1591549f
#define ONE_BY_THREE_PI			1.04719755f

extern const float elec_conv_ratio;

const float rad_to_deg;

extern float theta_m_prev;
extern float align_offset;				//offset angle IN RADIANS
extern int16_t cos_mid;
extern int16_t sin_mid;		//my encoder

float theta_abs_rad();
float theta_rel_rad();
float theta_abs_deg();
float theta_rel_deg();
float fmod_2pi(float in);

/*
 * fast trig functions
 */
float atan2_approx(float sinVal, float cosVal);
float sin_fast(float theta);
float cos_fast(float theta);

float unwrap(float theta,float * prev_theta);

#endif /* MAG_ENCODER_H_ */
