/*
 * mag-encoder.h
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */

#ifndef MAG_ENCODER_H_
#define MAG_ENCODER_H_
#include "adc.h"
#define ADC_SIN_CHAN 	5
#define ADC_COS_CHAN 	3

#define ONE_BY_THREE_FACTORIAL 	0.16666666666
#define ONE_BY_FIVE_FACTORIAL 	0.00833333333
#define HALF_PI 				1.57079632679
#define PI						3.14159265359
#define THREE_BY_TWO_PI     	4.71238898038
#define TWO_PI              	6.28318530718

const float rad_to_deg;

float align_offset;				//offset angle IN RADIANS
float theta_abs_rad();
float theta_rel_rad();
float theta_abs_deg();
float theta_rel_deg();

/*
 * fast trig functions
 */
float atan2_approx(float sinVal, float cosVal);
float sin_fast(float theta);
float cos_fast(float theta);

#endif /* MAG_ENCODER_H_ */
