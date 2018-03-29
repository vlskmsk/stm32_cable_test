/*
 * foc_commutation.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */

#ifndef FOC_COMMUTATION_H_
#define FOC_COMMUTATION_H_
#include "init.h"

int gl_current_input_offset;

typedef struct vect2
{
	float v1;
	float v2;
}vect2;

void conv_raw_current(float * i_a, float * i_b, float * i_c);
void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta);
void park_transform(float i_alpha, float i_beta, float theta, float * i_q, float * i_d);
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c);
void inverse_park_transform(float i_q, float i_d, float theta, float * i_alpha, float * i_beta);
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);
void sector_check(float alpha, float beta, float * theta, uint32_t * sector);


#endif /* FOC_COMMUTATION_H_ */
