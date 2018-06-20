/*
 * foc_commutation.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */

#ifndef FOC_COMMUTATION_H_
#define FOC_COMMUTATION_H_
#include "init.h"

#define CONST_E 0.367879441171442321595523770161460867445811131031767834507
float gl_I;
float gl_T;
float gl_V;


int16_t gl_current_input_offset;
float L;
float R;

typedef struct vect2
{
	float v1;
	float v2;
}vect2;
void init_observer();
void conv_raw_current(float * i_a, float * i_b, float * i_c);
void convert_phase_voltage(float * va, float * vb, float * vc);
void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta);
void park_transform(float i_alpha, float i_beta, float theta, float * i_q, float * i_d);
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c);
void inverse_park_transform(float i_q, float i_d, float theta, float * i_alpha, float * i_beta);
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);
void sector_check(float alpha, float beta, float * theta, uint32_t * sector);
float est_R();
float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2);
void obtain_encoder_offset();

#endif /* FOC_COMMUTATION_H_ */
