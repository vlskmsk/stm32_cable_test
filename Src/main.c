#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "delay_uS.h"

#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"

/*
 * TODO: Figure out how to quickly change dma-adc configuration to swap between
 * foc and trapezoidal motor
 *
 * main.c is getting a little crowded... clean this up
 */

//#define TEST_MODE
//#define GET_ALIGN_OFFSET
//#define DOWSE_ALIGN_OFFSET

#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5

float theta_enc = 0;
//extern uint8_t press_data_transmit_flag;

void align_offset_test();
float check_encoder_region();
void dowse_align_offset(float des_align_offset);
void sleep_reset();
void low_power_mode();



void slow_clock_8MHz();
void speedup_clock_48MHz();


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


volatile uint32_t time_exp;

int main(void)
{
	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_SPI3_Init();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_foc, NUM_ADC_FOC);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);

	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 100;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_SPI_TransmitReceive_DMA(&hspi3, t_data, r_data, NUM_SPI_BYTES);	//think need to change DMA settings to word from byte or half word
	HAL_UART_Receive_DMA(&huart1, uart_read_buffer, NUM_BYTES_UART_DMA );
	HAL_UART_DMAPause(&huart1);
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(1);

	/*
	 * TODO: configure for foc mode
	 */

	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	get_current_cal_offsets();

	TIMER_UPDATE_DUTY(0,0,0);

	//	obtain_encoder_midpoints();
#ifdef GET_ALIGN_OFFSET
	obtain_encoder_offset();
#elif defined(DOWSE_ALIGN_OFFSET)
	dowse_align_offset(HALF_PI);
#else
	//	align_offset = 2.51820064;				//offset angle IN RADIANS
	align_offset = HALF_PI;
#endif

	TIMER_UPDATE_DUTY(500,500,500);


	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	//	while(1);

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
	//	float comp_angle = check_encoder_region();
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	foc_vishan_lock_pos();
	mech_theta_prev = 0;
	foc_theta_prev = -TWO_PI;

#ifndef TEST_MODE

	while(1)
	{
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

		/***********************************Parse torque*************************************/
		//			uint32_t r_word = (r_data[1]<<24) | (r_data[2] << 16) | (r_data[3] << 8) | r_data[4];
		//			float * tmp = (float *)(&r_word);
		float * tmp = (float *)(&(r_data[1]));

		/**********load iq torque component, set id torque component for high speed**********/
		float iq_u = *tmp;
//		float id_u = iq_u * 6;
		float id_u = 0;

		/***********************limit iq and id to avoid overheating*************************/
		if(iq_u > 80)
			iq_u = 80;
		if(iq_u < -80)
			iq_u = -80;

		foc(iq_u,id_u);		//run foc!!!
		/******************************parse motor angle*************************************/
		float theta_m = unwrap(theta_abs_rad(), &mech_theta_prev);

		uint8_t * t_ptr = (uint8_t *)(&theta_m);
		int i;
		for(i=0;i<4;i++)
			t_data[i+1] = t_ptr[i];



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
 * Specialized function for VISHAN ONLY (only single pole pair motor with a variable align offset)
 */
void dowse_align_offset(float des_align_offset)
{
	float i_alpha,i_beta;
	uint32_t tA,tB,tC;
	inverse_park_transform(0, 0.3, 0, 1, &i_alpha, &i_beta);	//maybe call theta rel again?
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);		//TODO: since this produces (.2, -.1, -.1) -> (600, 450, 450), test (600, 400+50*sin(t), 400+50*sin(t)) and see if there
	while(1)
	{
		float theta_m = theta_abs_rad();
		float tol = .05;
		if(theta_m < des_align_offset+tol && theta_m > des_align_offset-tol)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	}
}




void slow_clock_8MHz()
{
	/*
	 * switch the MUX over to HSI direct, from PLL
	 */
	uint32_t cfgr_tmp;
	cfgr_tmp = RCC->CFGR;	//read cfgr register into temp register
	cfgr_tmp &= ~(0x3);		//clear the SW bits
	RCC->CFGR = cfgr_tmp;	//write to register
	cfgr_tmp = (RCC->CFGR & 0b1100) >> 2;	//read the SWS bits
	while(cfgr_tmp != 0b00)
	{
		cfgr_tmp = (RCC->CFGR & 0b1100) >> 2;	//wait until they match SW setting
	}

	/*
	 * disable the PLL
	 */
	uint32_t rcc_cr_tmp = RCC->CR;
	rcc_cr_tmp &= ~(1<<24);	//disable the PLL
	RCC->CR = rcc_cr_tmp;
	while( (RCC->CR & (1<<25)) != 0);	//wait until the PLL has fully stopped


	/*
	 * re-configure flash latency for 8MHz mode
	 */
	uint32_t FLatency = FLASH_LATENCY_0;
	/* Increasing the number of wait states because of higher CPU frequency */
	if(FLatency > (FLASH->ACR & FLASH_ACR_LATENCY))
	{
		/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
		__HAL_FLASH_SET_LATENCY(FLatency);

		/* Check that the new number of wait states is taken into account to access the Flash
	    memory by reading the FLASH_ACR register */
		if((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
	}

	/*
	 * TODO: re-configure UART clock source so that the baud rate is correct (to allow for pressure
	 * data reception while in lower-power mode)
	 */
	RCC_PeriphCLKInitTypeDef PeriphClkInit;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/*
 * roll back the changes from slow_down_clock()
 */
void speedup_clock_48MHz()
{

	/*
	 * TODO: roll back the UART clock settings made slow_clock
	 */

	/*
	 * first reconfigure flash latency for 48MHz
	 */
	uint32_t FLatency = FLASH_LATENCY_1;
	/* Increasing the number of wait states because of higher CPU frequency */
	if(FLatency > (FLASH->ACR & FLASH_ACR_LATENCY))
	{
		/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
		__HAL_FLASH_SET_LATENCY(FLatency);

		/* Check that the new number of wait states is taken into account to access the Flash
	    memory by reading the FLASH_ACR register */
		if((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
	}

	/*
	 * re-enable the PLL
	 */
	uint32_t rcc_cr_tmp = RCC->CR;
	rcc_cr_tmp |= (1<<24);	//enable the PLL
	RCC->CR = rcc_cr_tmp;

	while( (RCC->CR & (1<<25)) == 0);	//wait until pll is ready

	/*
	 * switch the MUX back to PLL source instead of direct from HSI
	 */
	uint32_t cfgr_tmp = RCC->CFGR;
	cfgr_tmp &= ~(0b11);	//clear the SW bits
	cfgr_tmp |= 0b10;		//set to PLL select
	RCC->CFGR = cfgr_tmp;	//write to register
	cfgr_tmp = (RCC->CFGR & 0b1100) >> 2;
	while(cfgr_tmp != 0b10)
	{
		cfgr_tmp = (RCC->CFGR & 0b1100) >> 2;	//wait until they match SW setting
	}

}








//
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  ** This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * COPYRIGHT(c) 2019 STMicroelectronics
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted provided that the following conditions are met:
//  *   1. Redistributions of source code must retain the above copyright notice,
//  *      this list of conditions and the following disclaimer.
//  *   2. Redistributions in binary form must reproduce the above copyright notice,
//  *      this list of conditions and the following disclaimer in the documentation
//  *      and/or other materials provided with the distribution.
//  *   3. Neither the name of STMicroelectronics nor the names of its contributors
//  *      may be used to endorse or promote products derived from this software
//  *      without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
//
//
//int main(void)
//{
//  HAL_Init();
//
//  SystemClock_Config();
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_ADC1_Init();
//  MX_TIM1_Init();
//  MX_USART1_UART_Init();
//  MX_SPI3_Init();
//  while (1)
//  {
//
//  }
//
//}
