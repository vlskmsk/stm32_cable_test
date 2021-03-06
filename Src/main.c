#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"
#include "foc-calibration.h"
#include "sleep.h"
//V7 R2 Hardware

//#define TEST_FOC
//#define CALIBRATE_MODE
//#define GET_ENCODER_MIDPOINTS

void low_power_mode();
void start_pwm();

volatile uint32_t time_exp;

uint8_t gl_gpio_evt_flag = 1;
uint32_t gl_fall_cnt = 0;
uint32_t gl_rise_cnt = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
		gl_rise_cnt++;
	else
		gl_fall_cnt++;
	gl_gpio_evt_flag = 1;
}

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
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 1);

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//enable BLDC Driver

	/* * Enable STM32 Center Aligned PWM, set output voltage to -100% on each phase, and initialize the CCR4 register for ADC injection * */
	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = TIM1->ARR-1;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	get_current_cal_offsets();

	/*******************************************ALIGN OFFSET!!! CRITICAL FOR FOC FUNCTIONALITY*******************************************************/
	align_offset = -5.50316763f;	//update for current sensing!!!
#ifdef CALIBRATE_MODE
	manual_align_calib();	//This is enabled for manual calibration of the encoder. Should not be used on a hand board driver, only on a single channel driver.
#endif
#ifdef TEST_FOC
	test_foc();				//Verification function. Made redundant by manual_align_calib().
#endif
#ifdef GET_ENCODER_MIDPOINTS
	obtain_encoder_midpoints();
#endif
	TIMER_UPDATE_DUTY(500,500,500);

	/*Anti lockup and encoder region alignment absorbed into a new force procedure.*/
	t_data[0] = BUSY_FORCE_ENCODER_REGION;
	if(check_motor_valid()==0x7)
		force_encoder_region();
	else
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN, 0);

	TIMER_UPDATE_DUTY(500,500,500);

	t_data[0] = MAIN_LOOP_READY;

	/*
	 * TODO: use faster (commented) atan2 and remove or significantly reduce compensator
	 */
	while(1)
	{
		handle_comms();

		/*After parsing I2C and SPI, perform motor control functions*/
		float theta_m = gl_theta_enc - m_q_offset;//unwrap(theta_abs_rad(), &mech_theta_prev) - m_q_offset;	//necessary to multiply internal offset by 2, because master expects format of 2*theta (from KMZ60 encoder)
		//mech_theta_prev = theta_m;

		tx_format.v = theta_m;	//in all cases, send position
		mcur_format.v = gl_iq_meas;

		switch(control_mode)
		{
		case CMD_CHANGE_IQ:
		{
			/***********************************Parse torque*************************************/
			/**********load iq torque component, set id torque component for high speed**********/
			float iq_u = rx_format.v;
			/***************limit iq and id to avoid overheating, run FOC************************/
			if(iq_u > iq_limit)
				iq_u = iq_limit;
			if(iq_u < -iq_limit)
				iq_u = -iq_limit;
			foc(iq_u);		//run foc!!!
			/******************************parse motor angle*************************************/
			break;
		}
		case CMD_CHANGE_POS:
		{
			/***********************************Parse position*************************************/
			float qd = rx_format.v;
			/**********position control maths**********/

			float err = (qd - theta_m*.5f*m_gear_ratio_conv);	//qd will be given to us in FINGER coordinates (i.e. degrees, assuming zero reference is full extension). Therefore, we must convert to these units
			float iq_u = err*kd_gain;	//master can change kd_gain with an SPI command
			/***********************limit iq and id to avoid overheating*************************/
			if(iq_u > iq_limit)
				iq_u = iq_limit;
			if(iq_u < -iq_limit)
				iq_u = -iq_limit;
			foc(iq_u);
			break;
		}
		case CMD_FORCE_ENCODER:
		{
			t_data[0] = BUSY_FORCE_ENCODER_REGION;
			force_encoder_region();	//this MUST be called outside of the handle_comms() function path!!! otherwise, you get infinite recursion and death
			TIMER_UPDATE_DUTY(500,500,500);
			t_data[0] = MAIN_LOOP_READY;
			control_mode = CMD_CHANGE_IQ;
			break;
		}
		default:
		{
			unwrap( theta_rel_rad(), &foc_theta_prev);	//no matter what, track the correct value of foc electrical theta
			break;
		}
		};
	}

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

void test_foc()
{
	uint32_t spin_time = 1000;
	while(1)
	{

		uint32_t ts = HAL_GetTick()+spin_time;
		while(HAL_GetTick() < ts)
		{
			foc(25);
		}
		ts = HAL_GetTick() + spin_time;
		while(HAL_GetTick() < ts)
		{
			foc(-25);
		}
	}
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
		if(gl_gpio_evt_flag == 1)
		{
			//first 5 bytes of r_data and t_data are RESERVED for motor control, and must not be overwritten
			for(int i = NUM_MOTOR_BYTES; i < (NUM_MOTOR_BYTES+num_uart_bytes); i++)
				t_data[i] = uart_read_buffer[i-5];

			gl_gpio_evt_flag = 0;
		}
	}

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway
}

