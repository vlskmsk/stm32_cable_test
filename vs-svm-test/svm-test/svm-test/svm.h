#ifndef SVM_H
#define SVM_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SQRT_3_BY_2 	0.866025404
#define ONE_BY_SQRT_3 	0.57735026919
#define TWO_BY_SQRT_3 	1.15470053838
#define SQRT_3 			1.7320508075688772935274463415f

int SVM(float alpha, float beta, float* tA, float* tB, float* tC);
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);
void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta);
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d);
void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta);

#endif