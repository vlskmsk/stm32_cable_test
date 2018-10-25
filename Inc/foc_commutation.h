/*
 * foc_commutation.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */

#ifndef FOC_COMMUTATION_H_
#define FOC_COMMUTATION_H_
#include "init.h"
#include "mag-encoder.h"

float foc_theta_prev;	//

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;


#define CONST_E 0.367879441171442321595523770161460867445811131031767834507

#define ONE_BY_SQRT_3 	0.57735026919
#define TWO_BY_SQRT_3 	1.15470053838
#define SQRT_3_BY_2 	0.866025404
#define SQRT_3 			1.7320508075688772935274463415f

float gl_I;
float gl_T;
float gl_V;


int32_t gl_current_input_offset_A;
int32_t gl_current_input_offset_B;
int32_t gl_current_input_offset_C;
float L;
float R;

typedef struct vect2
{
	float v1;
	float v2;
}vect2;

float foc(float iq_ref,float id_ref);

void get_current_cal_offsets();
void conv_raw_current(float * i_a, float * i_b, float * i_c);
void convert_phase_voltage(float * va, float * vb, float * vc);

void controller_PI(float i_q_ref, float i_q, float Kp, float Ki, float * x, float * u);

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta);
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d);
void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta);

int svm_sinusoidal(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);

//void init_observer();
//float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2);
//void sector_check(float alpha, float beta, float * theta, uint32_t * sector);
//float est_R();

void obtain_encoder_offset();
void obtain_encoder_midpoints();

#endif /* FOC_COMMUTATION_H_ */
