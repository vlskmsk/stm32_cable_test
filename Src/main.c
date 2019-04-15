#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"
#include "foc-calibration.h"
//V7 R2 Hardware

//#define TEST_FOC
//#define CALIBRATE_MODE
//#define GET_ENCODER_MIDPOINTS

void sleep_reset();
void low_power_mode();
void start_pwm();

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
	MX_USART1_UART_Init();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_foc, NUM_ADC_FOC);
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//enable BLDC Driver

	/* * Enable STM32 Center Aligned PWM, set output voltage to -100% on each phase, and initialize the CCR4 register for ADC injection * */
	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 950;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	get_current_cal_offsets();

	/*
	 * Stop mode recovery-> full speed FOC commutation confirmed. recovery now causes no performance hit.
	 * Repeated call functionality confirmed. 3 calls of sleep_reset with SystemClock config, and no impact of FOC performance.
	 *
	 * TODO: figure out why SPI interrupt recovery is not working (probably requires EVENT, which has been implemented already)
	 * i.e. does not clear WFI() from an SPI interrupt. Worst case, just use the SS pin as an EVENT pin or a GPIO interrupt.
	 */
	//	HAL_Delay(3000);
	//	sleep_reset();




	/*******************************************ALIGN OFFSET!!! CRITICAL FOR FOC FUNCTIONALITY*******************************************************/
	align_offset = -1.3;	//Currently all vishan motors will be given this (arbitrarily assigned) offset.
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

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);


	/******Anti-lockup measure. A simpler solution. If this doesn't work, attempt integral error approach***********/
	uint32_t init_twitch_ts=HAL_GetTick()+50;
	while(HAL_GetTick() < init_twitch_ts)
		foc(25,0);
	init_twitch_ts = HAL_GetTick()+25;
	while(HAL_GetTick() < init_twitch_ts)
		foc(-25,0);
	init_twitch_ts = HAL_GetTick()+20;
	while(HAL_GetTick() < init_twitch_ts)
		foc(0,0);
	/***********Exp. simple anti-lockup end*******/

	check_encoder_region();
	//	float theta_m_prev = foc_theta_prev;
	TIMER_UPDATE_DUTY(500,500,500);

	while(1)
	{
		/*Handle SPI Interrupt Packets*/
		HAL_SPI_TransmitReceive_IT(&hspi3, t_data, r_data, NUM_SPI_BYTES);
		if(new_spi_packet == 1)
		{
			parse_master_cmd();
			t_data[0] = 0;
			if(r_data[0] == CMD_CHANGE_PWM || r_data[0] == CMD_CHANGE_IQ)
				motor_update_ts = HAL_GetTick();	//
			new_spi_packet = 0;
		}

		/*TODO: Enable/test UART!!! This should be INTERRUPT based, not DMA based.*/
		if(new_uart_packet == 1)
		{
			handle_uart_buf();
			new_uart_packet = 0;
		}

		/*TODO: Trade this for STOP mode, the lowest power alternative.
		  TODO: Move this into the parse_master_cmd() function*/
		if(sleep_flag)
			low_power_mode();

		/*Once SPI and UART data has been parsed*/
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
			float qd = qd_format.v;
			/**********position control maths**********/

			float theta_m = unwrap(theta_abs_rad(), &mech_theta_prev) - m_q_offset*2;	//necessary to multiply internal offset by 2, because master expects format of 2*theta (from KMZ60 encoder)
			floatsend_t theta_transmit;
			theta_transmit.v = theta_m;
			for(i=0;i<4;i++)
				t_data[i+1] = theta_transmit.d[i];	//parse 2*theta into bytes to send over SPI

			float err = (qd - theta_m*.5*m_gear_ratio_conv);	//qd will be given to us in FINGER coordinates (i.e. degrees, assuming zero reference is full extension). Therefore, we must convert to these units
			float iq_u = err*kd_gain;	//master can change kd_gain with an SPI command
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
			foc(25,0);
		}
		ts = HAL_GetTick() + spin_time;
		while(HAL_GetTick() < ts)
		{
			foc(-25,0);
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
		if(new_uart_packet == 1)
		{
			handle_uart_buf();
			new_uart_packet = 0;
		}
	}

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway
}

/*
 * DESIRED BEHAVIOUR:
 * Call this function will disable the DRV8323, suspend all ADC, TIMER, and Systick interrupts, and place the STM32 in STOP mode. Upon recieving an interrupt
 * from SPI slave select, a WFI call from HAL_PWR_EnterSTOPMode will clear, the system clock will be re-initialized, and normal function will resume.
 *
 *
 * I have done successful tests of repeated calls of this function (by commenting out SuspendTick, WFI() gets cleared within 1ms of the call).
 *
 * With SystemClockConfig(), the uC is able to properly recover and commute an FOC motor at high speed. I did not directly measure speed
 * differences, but the audible tone and current draw was qualitatively correct at requested tau=+/-70. Additionally the uC current draw was the same.
 *
 * TODO: fix WFI recovery.
 */
void sleep_reset()
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	HAL_GPIO_WritePin(ENABLE_PORT,ENABLE_PIN,0);
	TIM1->CCER = (TIM1->CCER & DIS_ALL);
	HAL_TIM_Base_Stop(&htim1);
	HAL_SuspendTick();														//If you don't suspend the tick interrupt, WFI will clear within 1ms
	HAL_SPI_TransmitReceive_IT(&hspi3, t_data, r_data, NUM_SPI_BYTES);


	HAL_PWR_EnableBkUpAccess();
	HAL_PWR_DisableSleepOnExit();
	//		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);

	//TODO: re-enable PA15 as nSS hardware input
	SystemClock_Config();//systemclock configuration gets screwed up by STOPMode, so recover our settings (brute force)
	HAL_ResumeTick();														//fix what you tore down
	HAL_TIM_Base_Start(&htim1);
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;	//start_pwm();
	HAL_GPIO_WritePin(ENABLE_PORT,ENABLE_PIN,1);

}
