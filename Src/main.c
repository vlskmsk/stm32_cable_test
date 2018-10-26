#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, TRAPEZOIDAL_MODE} control_type;

//#define GET_ALIGN_OFFSET

#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	parse_master_cmd();
	execute_master_cmd();
}

float theta_enc = 0;

void align_offset_test();
float check_encoder_region();

void start_pwm();

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();
	//  MX_I2C1_Init();

//	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);


	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 750;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(1);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	get_current_cal_offsets();
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	//	init_observer();

	TIMER_UPDATE_DUTY(0,0,0);

	//	obtain_encoder_midpoints();
#ifdef GET_ALIGN_OFFSET
	obtain_encoder_offset();
#else
	align_offset = 1.03705454;				//offset angle IN RADIANS
#endif

	TIMER_UPDATE_DUTY(500,500,500);
	HAL_Delay(100);

	uint32_t led_ts = 0;

	uint32_t start_time = HAL_GetTick();

//	 * TODO: test lookup table with foc
//	 */
//	/*
//	 * TODO: verify that the lookup table is faster
//	 * 		2 methods:
//	 * 		put in closed loop and check current draw increase
//	 * 		use tim14 high resolution .16us timer for some number of consecuitve computations (i.e. time for 100 computations of a changing theta for both methods)
//	 */


//	align_offset_test();


/*****************************************************************************************/
	float toggle_state_ts = HAL_GetTick()+2000;
	float comp_angle = check_encoder_region();
	float theta_m_prev = comp_angle;
	foc_theta_prev = comp_angle;
	float theta_des = 1.5;
	while(1)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev);	//get the angle
		float u = (theta_des - theta_m)*.5;						//control law
		float id_u = u*2;

		if(u > 30)
			u = 30;
		if(u < -30)
			u = -30;
		if(id_u > 70)
			id_u = 70;
		if(id_u < -70)
			id_u = -70;

		foc(u,id_u);												//update foc
	}
/*****************************************************************************************/
	/*
	 * open loop ACUTAL sinusoidal
	 */
	while(1)
	{
		float theta = time_seconds()*300;
		theta = fmod_2pi(theta + PI) - PI;
		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		float i_alpha, i_beta;
		inverse_park_transform(.2, 0, sin_theta, cos_theta, &i_alpha, &i_beta);
		uint32_t tA,tB,tC;

//		svm_sinusoidal(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);

		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
	}

	uint8_t led_state;
	while(1)
	{
		uint32_t time = HAL_GetTick()-start_time;
		if(time < 2000)
		{
			foc(5,30);
		}
		else if (time >= 2000 && time < 4000)
		{
//			TIM1->CCR4 = 100;
			closedLoop(fw,forwardADCBemfTable,forwardEdgePolarity,400);
//			openLoop(fw, 200, 13000);
		}
		else if (time > 4000)
		{
			start_time = HAL_GetTick();
			TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
		}


		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}
}


void start_pwm()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


/*
 * check and correct the encoder region for KMZ60 encoder. Necessary because encoder returns
 * twice the actual angle
 */
float check_encoder_region()
 {
	uint32_t ts = HAL_GetTick();
	foc_theta_prev = theta_abs_rad();
	float theta_enc_start = unwrap( theta_rel_rad(), &foc_theta_prev)*.5;	//on your marks...
	float theta_enc_end = 0;	//get set
	while(HAL_GetTick() < ts + 50)
	{
		foc(12,0);
		theta_enc_end = unwrap( theta_rel_rad(), &foc_theta_prev)*.5;
	}
	if(theta_enc_end - theta_enc_start < 0)
		return -TWO_PI;
	else
		return 0;
}


float ao_pos = 0;
float ao_neg = 0;
void align_offset_test()
{
	float lower_limit = -PI/elec_conv_ratio;
	float upper_limit = -lower_limit;
	uint32_t period = 0;
	float theta_m_prev = 0;
	float theta_unwrapped_prev = 0;
	float theta_m;
	int prev_rotation_num = 0;
	uint32_t rotation_start_ts = HAL_GetTick();
	uint32_t min_pos_period = 0xFFFFFFFF;
	uint32_t min_neg_period = 0xFFFFFFFF;
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	for(align_offset = lower_limit; align_offset < upper_limit; align_offset+=.0005)
	{
		theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		float d_theta = theta_m - theta_unwrapped_prev;
		float wrapped_theta = theta_m * ONE_BY_TWO_PI;//= fmod_2pi(theta_m);
		int rotation_num = (int)wrapped_theta;
		if(rotation_num != prev_rotation_num)
		{
			period = HAL_GetTick()-rotation_start_ts;
			if(period < min_pos_period && d_theta > 0)
			{
				min_pos_period = period;
				ao_pos = align_offset;
			}
			if(period < min_neg_period && d_theta < 0)
			{
				min_neg_period = period;
				ao_neg = align_offset;
			}
		}
//		if(d_theta > 0.003)
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
//		else if(d_theta < -.003)


		foc(10,0);
		prev_rotation_num = rotation_num;
		theta_unwrapped_prev = theta_m;
	}
//	align_offset = ao_pos;
	align_offset = (ao_pos - ao_neg)*.5;
	TIMER_UPDATE_DUTY(0,0,0);
}



