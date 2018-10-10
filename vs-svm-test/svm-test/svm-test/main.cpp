#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "svm.h"


void main()
{

	float alpha, beta;
	float theta = 5.47;
	float sin_theta = sin(theta);
	float cos_theta = cos(theta);
	inverse_park_transform(.3, 0.0, sin_theta, cos_theta, &alpha, &beta);
	uint32_t tA, tB, tC;
	svm(alpha, beta, 1000, &tA, &tB, &tC);
	float tA_f, tB_f, tC_f;
	SVM(alpha, beta, &tA_f, &tB_f, &tC_f);
	tB_f *= 1000;
	tC_f *= 1000;
	tA_f *= 1000;
	
	tA_f = (uint32_t)tA_f;
	tB_f = (uint32_t)tB_f;
	tC_f = (uint32_t)tC_f;

	printf("tA = %d, tB = %d, tC = %d\r\n\r\n", tA, tB, tC);
	printf("tA = %f, tB = %f, tC = %f\r\n", tA_f, tB_f, tC_f);
	while (1);

}