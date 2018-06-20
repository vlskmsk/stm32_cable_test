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

float align_offset;				//offset angle IN RADIANS
float theta_abs_rad();
float theta_rel_rad();
float theta_abs_deg();
float theta_rel_deg();

#endif /* MAG_ENCODER_H_ */
