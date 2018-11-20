#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "sin_lookup.h"

/*
 * TODO: Figure out how to quickly change dma-adc configuration to swap between
 * foc and trapezoidal motor
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
extern uint8_t press_data_transmit_flag;

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


volatile uint32_t time_exp;

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init_TRAP();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_trap, NUM_ADC_TRAP);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);

	//	HAL_TIM_Base_Start(&htim14);
	//	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 100;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data, NUM_SPI_BYTES);	//think need to change DMA settings to word from byte or half word
	HAL_UART_Receive_DMA(&huart1, uart_read_buffer, 42);
	HAL_UART_DMAPause(&huart1);
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(1);

	/*
	 * Only valid for adc initialized for trap mode
	 */
	adc_init(TRAPEZOIDAL_MODE);
	gl_zero_cross_point = initZeroCrossPoint(dma_adc_trap);
	/*
	 * TODO: configure for foc mode
	 */
	adc_init(FOC_MODE);
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	get_current_cal_offsets();

	//	init_observer();

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

	uint8_t state = BRAKE;
	uint8_t prev_state = state;

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

//	adc_init(TRAPEZOIDAL_MODE);
//	slow_clock_8MHz();	//switch to HSI and turn off the PLL, thus dropping the current consumption to under 4.4ma
//	parse_master_cmd();
//	speedup_clock_48MHz();
//	while(1)
//	{
//		closedLoop(bw,backwardADCBemfTable,backwardEdgePolarity,1000);
//	}

	/*
	 * gl_rotorInterval is the time between consecutive control updates.
	 *
	 * For FOC, this number is the time between consecutive angle updates; specifically, theta_enc - foc_theta_prev
	 *
	 * For trapezoidal, this number is the time between consecutive 60 degree angle updates
	 *
	 * The time itself is in 6th's of a microsecond.
	 */
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
	adc_init(FOC_MODE);
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
	foc_vishan_lock_pos();
	mech_theta_prev = 0;
	foc_theta_prev = -TWO_PI;
	control_mode = FOC_MODE;
#ifndef TEST_MODE
	control_type prev_control_mode = control_mode;
	while(1)
	{
		if(new_spi_packet == 1)
		{
			parse_master_cmd();
			t_data[0] = control_mode;
			new_spi_packet = 0;
		}

		if(sleep_flag)
			low_power_mode();
		/*
		 * TODO: push the if statement below into a UART_PACKET flag, same as
		 * spi handler above.
		 */
		if(new_uart_packet == 1)
		{
			/*
			 * once we've recieved a flag that new uart data must be parsed, scan the double buffer for a complete data set
			 */
			for(int i = 0; i < 42; i++)
			{
				if (uart_read_buffer[i] == 's')
				{
					for(int j = 0; j < 21; j++)
					{
						pres_data[j] = uart_read_buffer[i+j];						//we can do outside of the handler
					}
					break;
				}
			}

			/*
			 * we've recieved a new round of data, so load it into the spi transmit buffer.
			 * the motor control spi protocol will carry it over to the master
			 */
			if(press_data_transmit_flag == 1)  //Bird
			{
				for(int i = 5; i < 26; i++)
				{
					t_data[i] = pres_data[i-5];
				}
			}

			new_uart_packet = 0;
		}

		switch(control_mode)
		{
		case FOC_MODE:
		{
			/*
			 * TODO: advanced transition logic forces transition to occur during only certain phases of closed loop
			 * OR
			 * tracks the number of mechanical rotations and RE-INITIALIZES the encoder!!!! (NOTE: this is the best mehtod, as it
			 * is valid for n pole pairs for all valid n)
			 */
			if(prev_control_mode != FOC_MODE)
			{
				adc_init(FOC_MODE);
				TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
				foc_vishan_lock_pos();
				foc_theta_prev = -TWO_PI;
			}
			/***********************************Parse torque*************************************/
			uint32_t r_word = (r_data[1]<<24) | (r_data[2] << 16) | (r_data[3] << 8) | r_data[4];
			float * tmp = (float *)(&r_word);

			/**********load iq torque component, set id torque component for high speed**********/
			float iq_u = *tmp;
			float id_u = iq_u * 6;

			/***********************limit iq and id to avoid overheating*************************/
			if(iq_u > 30)
				iq_u = 30;
			if(iq_u < -30)
				iq_u = -30;
			if(id_u > 75)
				id_u = 75;
			if(id_u < -75)
				id_u = -75;

			foc(iq_u,id_u);		//run foc!!!

			/******************************parse motor angle*************************************/
			float theta_m = unwrap(theta_abs_rad(), &mech_theta_prev);
			uint32_t * t_ptr = (uint32_t * )(&theta_m);
			uint32_t t_word = *t_ptr;
			t_data[1] = (t_word & 0xFF000000)>>24;
			t_data[2] = (t_word & 0x00FF0000)>>16;
			t_data[3] = (t_word & 0x0000FF00)>>8;
			t_data[4] = (t_word & 0x000000FF);

			gl_rotorPos = (int32_t)theta_m*0.954929659;//3/pi

			break;
		}
		case TRAPEZOIDAL_MODE:
		{

			if(prev_control_mode != TRAPEZOIDAL_MODE)
			{
				adc_init(TRAPEZOIDAL_MODE);
			}
			/*
			 * format
			 */
			if(gl_master_duty < -1000)
				gl_master_duty = -1000;
			else if (gl_master_duty > 1000)
				gl_master_duty = 1000;
			int duty = gl_master_duty;
			t_data[1] = (gl_rotorPos & 0xFF000000) >> 24;
			t_data[2] = (gl_rotorPos & 0x00FF0000) >> 16;
			t_data[3] = (gl_rotorPos & 0x0000FF00) >> 8;
			t_data[4] = (gl_rotorPos & 0x000000FF);

			if (duty > -MIN_BRAKE_DUTY && duty < MIN_BRAKE_DUTY)	//first, tighter condition
			{
				state = BRAKE;
				brake();
			}
			else if(duty > -MIN_ACTIVE_DUTY && duty < MIN_ACTIVE_DUTY)
			{
				state = STOP;
				stop();
			}
			else if (duty >= MIN_ACTIVE_DUTY)
			{

				if(duty < MIN_CLOSED_DUTY)
				{
					stop();
					state = FORWARD_OPEN;
				}
				else
				{
					if(prev_state == BACKWARD_CLOSED || prev_state == BACKWARD_OPEN)
						brake();

					if(prev_state != FORWARD_CLOSED)
						openLoopAccel(fw,forwardADCBemfTable, forwardEdgePolarity);

					closedLoop(fw,forwardADCBemfTable,forwardEdgePolarity,duty);
					state = FORWARD_CLOSED;
				}
			}
			else if (duty <= -MIN_ACTIVE_DUTY)
			{

				duty = -duty;
				if(duty < MIN_CLOSED_DUTY)
				{
					stop();
					state = BACKWARD_OPEN;
				}
				else
				{
					if(prev_state == FORWARD_CLOSED || prev_state == FORWARD_OPEN)
						brake();

					if(prev_state != BACKWARD_CLOSED)
						openLoopAccel(bw,backwardADCBemfTable,backwardEdgePolarity);

					closedLoop(bw,backwardADCBemfTable,backwardEdgePolarity,duty);
					state = BACKWARD_CLOSED;
				}
			}
			prev_state = state;

			break;
		}
		default:
			break;
		};
		prev_control_mode = control_mode;

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




void low_power_mode()
{

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway

	int i;
	for(i=0;i<5;i++)
		t_data[i] = 0xFF;				//set tx data to 0xFF to indicate low power mode has been entered

	slow_clock_8MHz();	//switch to HSI and turn off the PLL, thus dropping the current consumption to under 4.4ma

	/*
	 * We're awake, but we need to handle UART and SPI for the pressure sensor (even if we don't have a
	 * sensor, we can't kill the bus for the sensor that's active).
	 */
	while(sleep_flag)
	{
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
			/*
			 * once we've recieved a flag that new uart data must be parsed, scan the double buffer for a complete data set
			 */
			for(int i = 0; i < 42; i++)
			{
				if (uart_read_buffer[i] == 's')
				{
					for(int j = 0; j < 21; j++)
					{
						pres_data[j] = uart_read_buffer[i+j];						//we can do outside of the handler
					}
					break;
				}
			}

			/*
			 * we've recieved a new round of data, so load it into the spi transmit buffer.
			 * the motor control spi protocol will carry it over to the master
			 */
			if(press_data_transmit_flag == 1)  //Bird
			{
				for(int i = 5; i < 26; i++)
				{
					t_data[i] = pres_data[i-5];
				}
			}
			new_uart_packet = 0;
		}
	}
	speedup_clock_48MHz();

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);	//disable the gate driver. this is likely already done by the master, but might as well assert it anyway
	for(i=0;i<5;i++)
		t_data[i] = i;				//set tx data to 0xFF to indicate low power mode has been entered

	adc_init(TRAPEZOIDAL_MODE);
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

	HAL_TIM_Base_Stop(&htim14);
	HAL_TIM_Base_Stop(&htim1);
	HAL_ADC_Stop_DMA(&hadc);
	HAL_SPI_DMAStop(&hspi1);


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
