#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, TRAPEZOIDAL_MODE} control_type;

//#define GET_ALIGN_OFFSET

#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5

char msg_buf[32];
void print_string(char * c)
{
	int i;
	for(i=0; c[i] != 0; i++);
	HAL_UART_Transmit(&huart1, (uint8_t*)c, i, 100);
}
void print_int16(int16_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 2, 100);
}
void print_int32(int32_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	msg_buf[2] = (val & 0xFF0000)>>16;
	msg_buf[3] = (val & 0xFF000000)>>24;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 4, 100);
}

int led_state =0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	parse_master_cmd();
	execute_master_cmd();
}

int idx = 0;
volatile int total_delay = 0;

float gl_angle = 89;

typedef enum {ADVANCE_TO_ZERO = 0, REST_AT_ZERO = 1, ADVANCE_TO_ANGLE = 2, REST_AT_ANGLE = 3} bang_test_state;

volatile uint32_t t_l = 0;

float theta_enc = 0;

void start_pwm()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	//	MX_SPI1_Init();
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

	gl_angle = 0;

	TIMER_UPDATE_DUTY(0,0,0);

	//	obtain_encoder_midpoints();
#ifdef GET_ALIGN_OFFSET
	obtain_encoder_offset();
#else
	align_offset = 1.03717375;				//offset angle IN RADIANS
#endif

	TIMER_UPDATE_DUTY(500,500,500);
	HAL_Delay(100);

	uint32_t led_ts = 0;

	uint32_t start_time = HAL_GetTick();

//	while(1)
//	{
//		float theta = time_seconds()*300;
//		theta = fmod_2pi(theta + PI) - PI;
//		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
//		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
//		float i_alpha, i_beta;
//		inverse_park_transform(.12, 0, sin_theta, cos_theta, &i_alpha, &i_beta);
//		uint32_t tA,tB,tC;
//
//		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
//		TIMER_UPDATE_DUTY(tB,tA,tC);
//	}

	while(1)
	{
		foc(5,30);
	}

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
