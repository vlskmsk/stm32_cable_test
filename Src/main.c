#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"

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

int led_state =0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	parse_master_cmd();
	execute_master_cmd();
}

int idx = 0;
volatile int total_delay = 0;

float gl_angle = 89;


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

	init_23kHz_filt_coef();

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	//	int i;
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(100);

	gl_current_input_offset = (dma_adc_raw[ADC_CHAN_CURRENT_A]+dma_adc_raw[ADC_CHAN_CURRENT_B]+dma_adc_raw[ADC_CHAN_CURRENT_C])/3;

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	gl_zero_cross_point = initZeroCrossPoint(dma_adc_raw);


	init_observer();

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	float t = 0;

	//	int t_ts = HAL_GetTick();
	gl_angle = 0;
	float f_motor = 2*PI*10;
	float Va,Vb,Vc;
	float A = .2;

	//	int16_t val = 0;

	//	while(1)
	//	{
	//		float t = time_seconds();
	//
	//		val = (int16_t)(200.0*sin(t*2*M_PI*50));
	//
	//		msg_buf[0] = (val & 0x00FF);
	//		msg_buf[1] = (val & 0xFF00)>>8;
	//		HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 2, 100);
	//
	//	}


	//	float x1,x2;
	while(1)
	{
		float i_a,i_b,i_c , Va_m, Vb_m, Vc_m;
		conv_raw_current(&i_a,&i_b, &i_c);
		convert_phase_voltage(&Va_m,&Vb_m, &Vc_m);

		t = time_seconds();

		//		sprintf(msg_buf, "%d,%d\r\n", (int)(i_a*1000), (int)time_milliseconds());
		//		print_string(msg_buf);

		int32_t val1 = (int32_t)gl_current_input_offset-(int32_t)dma_adc_raw[ADC_CHAN_CURRENT_A];
		msg_buf[0] = (val1 & 0x00FF);
		msg_buf[1] = (val1 & 0xFF00)>>8;
		msg_buf[2] = (val1 & 0xFF0000)>>16;
		msg_buf[3] = (val1 & 0xFF000000)>>24;

		int32_t val2 = (int32_t)gl_current_input_offset-(int32_t)dma_adc_raw[ADC_CHAN_CURRENT_B];
		msg_buf[4] = (val2 & 0x00FF);
		msg_buf[5] = (val2 & 0xFF00)>>8;
		msg_buf[6] = (val2 & 0xFF0000)>>16;
		msg_buf[7] = (val2 & 0xFF000000)>>24;

//		int16_t val3 = (int16_t)(i_c*500);
		int32_t val3 = (int32_t)gl_current_input_offset-(int32_t)dma_adc_raw[ADC_CHAN_CURRENT_C];
		msg_buf[8] = (val3 & 0x00FF);
		msg_buf[9] = (val3 & 0xFF00)>>8;
		msg_buf[10] = (val3 & 0xFF0000)>>16;
		msg_buf[11] = (val3 & 0xFF000000)>>24;

		HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 12, 10);

		//		f_motor = 3;
		//		float theta = cos(t*f_motor)*12*M_PI * 5;
		//
		//		Va = A*sin(theta);
		//		Vb = A*sin(theta + 2*M_PI/3);
		//		Vc = A*sin(theta + 4*M_PI/3);
		//		t = 0;
		Va = A*sin(f_motor*t);
		Vb = A*sin(f_motor*t + 2*M_PI/3);
		Vc = A*sin(f_motor*t + 4*M_PI/3);

		if(f_motor >= 2*M_PI*100)
			f_motor = 2*M_PI*100;


		float Valpha, Vbeta;
		uint32_t tA,tB,tC;
		clarke_transform(Va,Vb,Vc,&Valpha, &Vbeta);
		svm(Valpha, Vbeta, TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);


		if(TIM14_ms()>=led_ts)
		{
			//			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = TIM14_ms() + 200;
		}
	}

}

