#include "svm.h"


/*
* procedure:
* First divide (alpha,beta) vector into 4 quadrants.
* Then, for each quadrant, check which sector. two possibilities for each.
* once sectors are parsed, convert to times.
*/
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC)
{
	float pwm_half_period = (float)pwm_period_cnt;
	uint32_t sector;
	if (beta >= 0.0f)	//quadrant 1 or two
	{
		if (alpha >= 0.0f)	//quadrant 1
		{
			if (beta <= alpha*SQRT_3)	//sector 1
				sector = 1;
			else							//sector 2
				sector = 2;
		}
		else				//quadrant 2
		{
			if (beta <= alpha*-SQRT_3)	//sector 2
				sector = 3;
			else							//sector 3
				sector = 2;
		}
	}
	else			//quadrant 3 or 4
	{
		if (alpha >= 0.0f)	//quadrant 4
		{
			if (beta < -SQRT_3*alpha)
				sector = 5;
			else
				sector = 6;
		}
		else				//quadrant 3
		{
			if (beta < alpha*SQRT_3)
				sector = 5;
			else
				sector = 4;
		}
	}

	switch (sector)
	{
		case 1:
		{
			uint32_t t1 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t2 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
			*tA = (pwm_half_period - t1 - t2) * .5;
			*tB = *tA + t1;
			*tC = *tB + t2;
			break;
		}
		case 2:
		{
			uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
			*tB = (pwm_half_period - t2 - t3) * .5;
			*tA = *tB + t3;
			*tC = *tA + t2;
			break;
		}
		case 3:
		{
			uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
			*tB = (pwm_half_period - t3 - t4) * .5;
			*tC = *tB + t3;
			*tA = *tC + t4;
			break;
		}
		case 4:
		{
			uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
			*tC = (pwm_half_period - t4 - t5) * .5;
			*tB = *tC + t5;
			*tA = *tB + t4;

			break;
		}
		case 5:
		{
			uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
			*tC = (pwm_half_period - t5 - t6) * .5;
			*tA = *tC + t5;
			*tB = *tA + t6;
			break;
		}
		case 6:
		{
			uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
			uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
			*tA = (pwm_half_period - t6 - t1) * .5;
			*tC = *tA + t1;
			*tB = *tC + t6;
			break;
		}
	}
	return sector;
}

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta)
{
	(*i_alpha) = i_a;
	*i_beta = ONE_BY_SQRT_3*i_a + TWO_BY_SQRT_3*i_b;
}
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d)
{
	*i_d = i_alpha*cos_theta + i_beta*sin_theta;
	*i_q = -i_alpha*sin_theta + i_beta*cos_theta;
}

void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c)
{
	*i_a = i_alpha;
	*i_b = -.5*i_alpha + SQRT_3_BY_2*i_beta;
	*i_c = -.5*i_alpha - SQRT_3_BY_2*i_beta;
}

void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta)
{
	*i_alpha = i_d*cos_theta - i_q*sin_theta;
	*i_beta = i_d*sin_theta + i_q*cos_theta;
}






int SVM(float alpha, float beta, float* tA, float* tB, float* tC) 
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (ONE_BY_SQRT_3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-ONE_BY_SQRT_3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-ONE_BY_SQRT_3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (ONE_BY_SQRT_3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - ONE_BY_SQRT_3 * beta;
            float t2 = TWO_BY_SQRT_3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;

            break;
        }
        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + ONE_BY_SQRT_3 * beta;
            float t3 = -alpha + ONE_BY_SQRT_3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;

            break;
        }
        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = TWO_BY_SQRT_3 * beta;
            float t4 = -alpha - ONE_BY_SQRT_3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;

            break;
        }
        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + ONE_BY_SQRT_3 * beta;
            float t5 = -TWO_BY_SQRT_3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;

            break;
        }
        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - ONE_BY_SQRT_3 * beta;
            float t6 = alpha - ONE_BY_SQRT_3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;

            break;
        }
        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -TWO_BY_SQRT_3 * beta;
            float t1 = alpha + ONE_BY_SQRT_3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;

            break;
        }
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}