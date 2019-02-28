#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "delay_uS.h"

#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"

//V7 R2 Hardware

/*
 * TODO: Figure out how to quickly change dma-adc configuration to swap between
 * foc and trapezoidal motor
 *
 * main.c is getting a little crowded... clean this up
 */

//#define TEST_MODE
//#define GET_ALIGN_OFFSET
//#define DOWSE_ALIGN_OFFSET
//#define TEST_FOC
//#define CALIBRATE_MODE

#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5

float theta_enc = 0;
//extern uint8_t press_data_transmit_flag;

void align_offset_test();
void check_encoder_region();
void dowse_align_offset();
void sleep_reset();
void low_power_mode();
void test_foc();


/*
 * Quickly align the encoder in the correct position. Too fast for correct align offset calculation, but fast enough to spin in the right direction
 * NOTE: some other method is necessary for closed->foc. Current plan is to track closed state/step, and force transition to occur if the step is in the valid half
 *
 * NOTE: given n pole pairs, this strategy is only valid for n=1. n>1 has n mechanical angles for 1 locked electrical angle, therefore the region is not guaranteed.
 */
void foc_vishan_lock_pos()
{
	float i_beta,i_alpha;
	uint32_t tA,tB,tC;
	inverse_park_transform(0, 0.3, 0, 1, &i_alpha, &i_beta);	//maybe call theta rel again?
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);		//TODO: since this produces (.2, -.1, -.1) -> (600, 450, 450), test (600, 400+50*sin(t), 400+50*sin(t)) and see if there
	HAL_Delay(15);
}

/*
 * same as writing to CCER register. this is done in one line elsewhere in the code
 */
void start_pwm()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	//	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
}

/*
 * Checks a candidate align_offset (global variable, used by the encoder)
 * by producing a triangle speed waveform using constant positive and negative
 * torques; i.e. positive tau_ref for run_time ms, negative tau_ref  for run_time ms.
 * The function measures the position at the start of the waveform, the peak/tip of the
 * triangle, and the endpoint. Since the torque is constant, the speed is determined by physical
 * system (same conditions for both directions in an unloaded motor) and the quality of the align
 * offset; therefore, for a good offset, the error between the start and endpoints should be minimal.
 *
 * However, some align offsets produce ZERO speed since they're so terrible. To factor this in, the
 * midpoint is compared to the starting point; if the motor has not moved appreciably (a generous 1 radian)
 * then the align offset is no good, and an arbitrary large error is returned.
 *
 *In general, the align offset which produces the smallest error that is LESS than a given threshold (1 rad or less)
 *should be accepted.
 *
 *For motors with fixed encoder mounts, this function should be used in tandem with sweep_align.
 *for motors with variable encoder mounts (and fixed align offset values) this function should
 *relay visual feedback to a human operator, through the light and potentially a matlab plot.
 */
#define OUT_OF_BOUNDS_ERROR 1000000
volatile float gl_error_print = 0;
uint32_t print_ts=0;
float check_align_offset(uint32_t run_time, float tau_ref)
{
	uint32_t run_ts = HAL_GetTick() + run_time;
	float init_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get the initial motor position
	float prev_theta = 0;//need to track a second time for speed. unwrap smashes it
	uint8_t dir_correct_flag = 1;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta < 0)	//if speed is negative and direction is positive
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(tau_ref,0);
	}

	/*make sure we've picked an align offset that's not completely out of bounds, by checking that we've actually traveled an appreciable distance during phase 1*/
	float mid_check_err = init_pos - prev_theta;
	//	mid_check_err &= ~0x80000000;
	if(mid_check_err < 0)
		mid_check_err = -mid_check_err;
	if(mid_check_err < 1)	//if the motor didn't move during this sweep
	{
		gl_error_print = OUT_OF_BOUNDS_ERROR;
		return gl_error_print;
	}

	run_ts = HAL_GetTick() + run_time;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta > 0)	//if speed is positive and direction is negative
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(-tau_ref,0);
	}

	/*First, check to make sure you passed all edge condition flags*/
//	if(dir_correct_flag == 0)
//	{
//		gl_error_print = OUT_OF_BOUNDS_ERROR;
//		return gl_error_print;	//if direction mismatch, return an out of bounds error.
//	}

	float final_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
	float err = final_pos - init_pos;
	if(err < 0)
		err = -err;
	gl_error_print = err;
	return err;
}

void manual_align_calib()
{
	while(1)
	{
		float err = check_align_offset(300, 20);
		if(err < PI)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	}
}

volatile uint32_t time_exp;

int main(void)
{

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_foc, NUM_ADC_FOC);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);

	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 950;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(1);

	/*
	 * TODO: configure for foc mode
	 */

	//	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	get_current_cal_offsets();

	TIMER_UPDATE_DUTY(0,0,0);

	//	//	obtain_encoder_midpoints();

#ifdef GET_ALIGN_OFFSET
	obtain_encoder_offset();
#elif defined(DOWSE_ALIGN_OFFSET)
	dowse_align_offset(HALF_PI);
#else
	//	align_offset = 2.51820064;				//offset angle IN RADIANS
	//	align_offset = -2.24159265359;
	//align_offset = -0.025;	//we got a problem
	align_offset = -1.3;	//gonna keep this arbitrarily and switch over to dev on the iq match method. hopefully.
	//	align_offset = 3.0368185;
#endif
#ifdef CALIBRATE_MODE
	manual_align_calib();
#endif
#ifdef TEST_FOC
	test_foc();
#endif

	TIMER_UPDATE_DUTY(500,500,500);


	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	//	align_offset_test();
	/*****************************************************************************************/
	//	float comp_angle = check_encoder_region();
	//	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	check_encoder_region();
	//	float theta_m_prev = foc_theta_prev;
	TIMER_UPDATE_DUTY(500,500,500);

#ifndef TEST_MODE

	while(1)
	{
		HAL_SPI_TransmitReceive_IT(&hspi3, t_data, r_data, NUM_SPI_BYTES);
		if(new_spi_packet == 1)
		{
			parse_master_cmd();
			t_data[0] = 0;
			if(r_data[0] == CMD_CHANGE_PWM || r_data[0] == CMD_CHANGE_IQ)
				motor_update_ts = HAL_GetTick();	//
			new_spi_packet = 0;
		}

		if(new_uart_packet == 1)
		{
			handle_uart_buf();
			new_uart_packet = 0;
		}

		if(sleep_flag)
			low_power_mode();

		switch(control_mode)
		{
		case CMD_CHANGE_IQ:
		{
			/***********************************Parse torque*************************************/
			floatsend_t tau_format;
			int i;
			for(i=0;i<4;i++)
				tau_format.d[i] = r_data[i+1];
			/**********load iq torque component, set id torque component for high speed**********/
			float iq_u = tau_format.v;
			float id_u = 0;
			/***************limit iq and id to avoid overheating, run FOC************************/
			if(iq_u > 80)
				iq_u = 80;
			if(iq_u < -80)
				iq_u = -80;
			foc(iq_u,id_u);		//run foc!!!
			/******************************parse motor angle*************************************/
			float theta_m = unwrap(theta_abs_rad(), &mech_theta_prev);
			floatsend_t theta_transmit;
			theta_transmit.v = theta_m;
			for(i=0;i<4;i++)
				t_data[i+1] = theta_transmit.d[i];
			break;
		}
		case CMD_CHANGE_POS:
		{
			/***********************************Parse position*************************************/
			floatsend_t qd_format;
			int i;
			for(i=0;i<4;i++)
				qd_format.d[i] = r_data[i+1];
			/**********position control maths**********/
			float qd = qd_format.v;

			float theta_m = unwrap(theta_abs_rad(), &mech_theta_prev) - m_q_offset*2;
			floatsend_t theta_transmit;
			theta_transmit.v = theta_m;
			for(i=0;i<4;i++)
				t_data[i+1] = theta_transmit.d[i];

			float err = (qd - theta_m*.5*0.134497135);
			float iq_u = err*kd_gain;	//TODO: make this global kd, add to spi protocol
			/***********************limit iq and id to avoid overheating*************************/
			if(iq_u > 80)
				iq_u = 80;
			if(iq_u < -80)
				iq_u = -80;
			foc(iq_u,0);
			break;
		}

		default:
			break;
		};


	}
#else
	/*****************************************************************************************/
	uint32_t t_ts = HAL_GetTick();
	while(1)
	{
		adc_init(FOC_MODE);
		TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
		foc_vishan_lock_pos();
		foc_theta_prev = -TWO_PI;

		t_ts = HAL_GetTick()+1000;
		while(HAL_GetTick() < t_ts)
			foc(5,30);

		adc_init(TRAPEZOIDAL_MODE);
		t_ts = HAL_GetTick()+1000;
		while(HAL_GetTick() < t_ts)
			closedLoop(fw,forwardADCBemfTable,forwardEdgePolarity,1000);
	}
#endif

}




float ao_pos = 0;
float ao_neg = 0;
void align_offset_test()
{
	float lower_limit = -PI/elec_conv_ratio;
	float upper_limit = -lower_limit;
	uint32_t period = 0;
	theta_m_prev = 0;
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


/*TODO: implement low power mode on f301*/
void low_power_mode()
{

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway

	int i;
	for(i=0;i<5;i++)
		t_data[i] = 0xFF;				//set tx data to 0xFF to indicate low power mode has been entered

	/*
	 * We're awake, but we need to handle UART and SPI for the pressure sensor (even if we don't have a
	 * sensor, we can't kill the bus for the sensor that's active).
	 */
	while(sleep_flag)
	{
		__WFI();
		/*
		 *	handle new spi buffer
		 */
		if(new_spi_packet == 1)
		{
			parse_master_cmd();
			new_spi_packet = 0;
		}

		/*
		 * Handle new uart double buffer
		 */
		if(new_uart_packet == 1)
		{
			handle_uart_buf();
			new_uart_packet = 0;
		}
	}

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway
}

/*
 * TODO: 	1. figure out how to recover from stop mode
 * 			2. configure SPI gpio pins as Hi-z so
 * 				they don't screw with the sensors that are active
 *
 * If this works and doesn't interfere with SPI, then in the long term we'll
 * use this for drivers that aren't using pressure sensors, and low_power_mode
 * for sensors that need to carry sensor data for the ultimate low power consumption
 *
 */
void sleep_reset()
{

	/*
	 * Tear down, set up for stop
	 */
	TIM1->CCER = (TIM1->CCER & DIS_ALL);
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);

	HAL_GPIO_WritePin(STAT_PORT, STAT_PIN, 0);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);

	//	HAL_TIM_Base_Stop(&htim14);
	HAL_TIM_Base_Stop(&htim1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_SPI_DMAStop(&hspi3);


	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
	NVIC_SystemReset();
	/*Everything below this statement will not get executed.
	 * Reason for the reset statement is because for some reason
	 * exiting stop mode causes the cpu to run ridiculously slow.
	 * THIS IS A HACK/WORKAROUND, what should replace it is
	 * returning the system settings to normal and entering the og loop
	 */
	sleep_flag = 0;
}



/*
 *
 */
void check_encoder_region()
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t ts = HAL_GetTick();
	theta_m_prev = 0;
	float theta_set = (unwrap(theta_abs_rad(), &theta_m_prev)*.5)+.4;	//get the initial position and use it to set the motor setpoint.
	float err = 0;
	uint32_t try_ts = 100;
	uint32_t off_ts = 0;
	while(1)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get current motor position
		err = theta_set-theta_m;		// and error
		float tau = 30*err;				//position control
		if(tau > 50)
			tau = 50;
		if(tau < -50)
			tau = -50;
		foc(tau,0);

		if(err < 0)
			err = -err;		//get the absolute value of the error

		if(err > .6 && HAL_GetTick() > ts)	//if the error is great and you haven't tried for some time, reverse the direction
		{
			foc_theta_prev -= TWO_PI;
			ts = HAL_GetTick()+try_ts;
			try_ts+=50;	//if you failed on the last attempt, try for just a liiitle bit longer. This improves stability (i.e. no infinite oscillations if you're significantly out of bounds)
		}

		if(err > .1)	//if the error is great,
			off_ts = HAL_GetTick();	//set the timestamp
		else
		{
			if(HAL_GetTick() - off_ts > 50)	//if you've settled (low error for an acceptable time) break
			{
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
				break;
			}
		}
	}
}

void dowse_align_offset()
{
	HAL_Delay(1000);
	uint32_t spin_time = 1000;
	while(1)
	{
		for(align_offset = -PI; align_offset < PI; align_offset += .1)
		{
			uint32_t ts = HAL_GetTick()+spin_time;
			while(HAL_GetTick() < ts)
			{
				foc(25,0);
			}
			ts = HAL_GetTick() + spin_time;
			while(HAL_GetTick() < ts)
			{
				foc(-25,0);
			}
		}
		HAL_Delay(3000);
	}
}
void test_foc()
{
	uint32_t spin_time = 1000;
	while(1)
	{

		uint32_t ts = HAL_GetTick()+spin_time;
		while(HAL_GetTick() < ts)
		{
			foc(25,0);
		}
		ts = HAL_GetTick() + spin_time;
		while(HAL_GetTick() < ts)
		{
			foc(-25,0);
		}
	}
}




