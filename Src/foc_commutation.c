/*
 * foc_commutation.c
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */
#include "foc_commutation.h"
#include "adc.h"
#include "commutation.h"
#include <math.h>
#include "delay_uS.h"
float mL = 0;
float mR = 0;
float V_psi = 0;
float L;
float R;

#define ONE_BY_SQRT_3 	0.57735026919
#define TWO_BY_SQRT_3 	1.15470053838
#define SQRT_3_BY_2 	0.866025404

#define ADC_CURRENT_CONV_RATIO 0.002877371651785714285714	//	= (3.3/4096)/(40*.007)
//#define ADC_CURRENT_CONV_RATIO 0.00421428571	//adjustment calibrated to red handheld DMM

void init_observer()
{
	//for vishan:
	//Resistance of a single coil to CT = 0.86
	//Inductance (measured from scope) of a SINGLE COIL to CT = 1.462
	//VpHz = .01502703????????? (unknown) (was not able to measure via vesc)

	//	est_R();
	R = .86;			//R (single coil)
	L = 0.000001462;	//L (single coil)
	V_psi = .0012655;
}

/*
 * based on http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5200471
 */
float gl_t_prev = 0;
float gl_t = 0;
float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2)
{
	gl_t_prev = gl_t;
	gl_t = (float)((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT))*seconds_per_tick;
	float dt = gl_t-gl_t_prev;	//TODO: estimate dt

	float y1,y2;
	y1 = -R*i_a+v_a;
	y2 = -R*i_b+v_b;

	float gamma = 1;	//find later
	float L_i_a = L*i_a;
	float L_i_b = L*i_b;
	float n1,n2;
	n1 = *x1-L_i_a;
	n2 = *x2-L_i_b;
	float e = V_psi*V_psi - (n1*n1+n2*n2);
	float xdot1,xdot2;
	xdot1 = y1 + gamma*.5 + n1*e;
	xdot2 = y2 + gamma*.5 + n2*e;
	*x1 += xdot1*dt;
	*x2 += xdot2*dt;

	/*
	 * TODO: try putting the observer in a loop, as in vesc.
	 */

	return atan2((*x2-L_i_b),(*x1-L_i_a));

}

#define INHA 0
#define INLA 1
#define INHB 2
#define INLB 3
#define INHC 4
#define INLC 5

int gl_sector=1;

/*
 *	convert each phase adc voltage to actual current value
 */
void conv_raw_current(float * i_a, float * i_b, float * i_c)
{
//	*i_a = (float)(gl_current_input_offset - dma_adc_raw[ADC_CHAN_CURRENT_A])*ADC_CURRENT_CONV_RATIO;
	*i_a = (float)(gl_current_input_offset - iA_filt)*ADC_CURRENT_CONV_RATIO;
	*i_b = (float)(gl_current_input_offset - dma_adc_raw[ADC_CHAN_CURRENT_B])*ADC_CURRENT_CONV_RATIO;
	*i_c = (float)(gl_current_input_offset - dma_adc_raw[ADC_CHAN_CURRENT_C])*ADC_CURRENT_CONV_RATIO;

	uint8_t state = 0x7 & ((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10) << 2) | (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) << 1) | HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8));
//	uint8_t state = shunt_state;
	/*
	 * NOTE: test out setting shunt state variable in the handler, at a decimated sampling frequency.
	 */

	// c , b , a
	switch (state)
	{
	case 1:
	{
		*i_a = -(*i_b + *i_c);
		break;
	}
	case 2:
	{
		*i_b = -(*i_a + *i_c);
		break;
	}
	case 3:
	{
		*i_a = *i_c*-.5;
		*i_b = *i_a;
		break;
	}
	case 4:
	{
		*i_c = -(*i_a + *i_b);
		break;
	}
	case 5:
	{
		*i_a = *i_b*-.5;
		*i_c = *i_a;
		break;
	}
	case 6:
	{
		*i_b = *i_a*-.5;
		*i_c = *i_b;
		break;
	}
	}
}

void convert_phase_voltage(float * va, float * vb, float * vc)
{
	*va = (float)dma_adc_raw[ADC_CHAN_BEMF_A] * 0.00261619718;
	*vb = (float)dma_adc_raw[ADC_CHAN_BEMF_B] * 0.00261619718;
	*vc = (float)dma_adc_raw[ADC_CHAN_BEMF_C] * 0.00261619718;
}

#define SQRT_3 1.7320508075688772935274463415f

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
	if(beta >= 0.0f)	//quadrant 1 or two
	{
		if(alpha >= 0.0f)	//quadrant 1
		{
			if(beta <= alpha*SQRT_3)	//sector 1
				sector = 1;
			else							//sector 2
				sector = 2;
		}
		else				//quadrant 2
		{
			if(beta <= alpha*-SQRT_3)	//sector 2
				sector = 3;
			else							//sector 3
				sector = 2;
		}
	}
	else			//quadrant 3 or 4
	{
		if(alpha >= 0.0f)	//quadrant 4
		{
			if(beta < -SQRT_3*alpha)
				sector = 5;
			else
				sector = 6;
		}
		else				//quadrant 3
		{
			if(beta < alpha*SQRT_3)
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
		*tA = (pwm_half_period - t1 - t2) / 2;
		*tB = *tA + t1;
		*tC = *tB + t2;
		break;
	}
	case 2:
	{
		uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t2 - t3) / 2;
		*tA = *tB + t3;
		*tC = *tA + t2;
		break;
	}
	case 3:
	{
		uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t3 - t4) / 2;
		*tC = *tB + t3;
		*tA = *tC + t4;
		break;
	}
	case 4:
	{
		uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t4 - t5) / 2;
		*tB = *tC + t5;
		*tA = *tB + t4;

		break;
	}
	case 5:
	{
		uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t5 - t6) / 2;
		*tA = *tC + t5;
		*tB = *tA + t6;
		break;
	}
	case 6:
	{
		uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t6 - t1) / 2;
		*tC = *tA + t1;
		*tB = *tC + t6;
		break;
	}
	}
	gl_sector = sector;
	return sector;
}

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta)
{
	(*i_alpha) = i_a;
	*i_beta = ONE_BY_SQRT_3*i_a + TWO_BY_SQRT_3*i_b;
}
void park_transform(float i_alpha, float i_beta, float theta, float * i_q, float * i_d)
{
	*i_d = i_alpha*cos(theta)+i_beta*sin(theta);
	*i_q = -i_alpha*sin(theta)+i_beta*cos(theta);
}
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c)
{
	*i_a = i_alpha;
	*i_b = -.5*i_alpha + SQRT_3_BY_2*i_beta;
	*i_c = -.5*i_alpha - SQRT_3_BY_2*i_beta;
}
void inverse_park_transform(float i_q, float i_d, float theta, float * i_alpha, float * i_beta)
{
	*i_alpha = i_d*cos(theta) - i_q*sin(theta);
	*i_beta = i_d*sin(theta) + i_q*cos(theta);
}

/*
 * Seems to work... not particularly accurate unfortunately
 */
float est_R()
{
	TIMER_UPDATE_DUTY(0,0,0);
	TIMER_UPDATE_DUTY(1000,1000,0);
	HAL_Delay(10);
	mR = 0;
	int i;
	for(i=0;i<10;i++)
	{
		float Va = (float)dma_adc_raw[ADC_CHAN_BEMF_A] * 0.000805664062 * 3.24725565716;
		float Vb = (float)dma_adc_raw[ADC_CHAN_BEMF_B] * 0.000805664062 * 3.24725565716;
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);
		mR += (((Va+Vb)*.5)/i_c)/1.5;
	}
	mR = mR/(float)i;
	TIMER_UPDATE_DUTY(0,0,0);
	R = mR;
	return R;
}


/*
 * TODO: FIX THIS!!! general bones of getting L from time constant, but it does not work for the vishan. Tested with the
 * hobby, and didn't work either. should try shunt+ scope and measure the T that way.
 */
float gl_I;
float gl_T;
float gl_V = 7.4;

float est_L()
{
	float coil_r = R;

	TIMER_UPDATE_DUTY(0,0,0);
	HAL_Delay(100);
	TIMER_UPDATE_DUTY(1000,1000,1000);
	HAL_Delay(1);
	float Va_m = (float)dma_adc_raw[ADC_CHAN_BEMF_A] * 0.000805664062 * 3.24725565716;
	float Vb_m = (float)dma_adc_raw[ADC_CHAN_BEMF_B] * 0.000805664062 * 3.24725565716;
	float Vc_m = (float)dma_adc_raw[ADC_CHAN_BEMF_C] * 0.000805664062 * 3.24725565716;
	gl_V = (Va_m+Vb_m+Vc_m)/3.0;
	float thresh = (gl_V/(2*coil_r))*.37;

	while(1)
	{
		TIMER_UPDATE_DUTY(1000,1000,0);
		uint32_t t_init = TIM14_ms()*1000+TIM14->CNT;
		TIM14->CNT = 0;
		while(1)	//might be the case that uS is not enough reso. for this kind of test
		{
			float i_a,i_b,i_c;
			conv_raw_current(&i_a,&i_b, &i_c);
			if(i_c > thresh)
			{
				gl_I = i_c;
				gl_T = (float)((TIM14_ms()*1000+TIM14->CNT)-t_init);
				break;
			}
			if(TIM14->CNT > 600)
				break;
		}
		TIMER_UPDATE_DUTY(0,0,0);
		printf("T found");
		while(1);
		//		float max_cur = coil_r
	}
	return 0;
}
