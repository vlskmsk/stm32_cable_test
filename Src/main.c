#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
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
	TIM1->CCR4 = 700;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle


	//	init_23kHz_filt_coef();

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	//	int i;
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(100);

	//	gl_current_input_offset = (dma_adc_raw[ADC_CHAN_CURRENT_A]+dma_adc_raw[ADC_CHAN_CURRENT_B]+dma_adc_raw[ADC_CHAN_CURRENT_C])/3;
	get_current_cal_offsets();


	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	//	gl_zero_cross_point = initZeroCrossPoint(dma_adc_raw);


	init_observer();

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	float t = 0;

	//	int t_ts = HAL_GetTick();
	gl_angle = 0;
	float f_motor = 2*PI*1;
	float Va,Vb,Vc;
	float A = .09;


	TIMER_UPDATE_DUTY(0,0,0);
	float theta = 0;
	float tstart = time_seconds();
	float f = 2*M_PI;

	bang_test_state bang_state = ADVANCE_TO_ZERO;
	int angle_set = 0;
	int angle_inc = 90;
	obtain_encoder_offset();

	/*
		while(1)
		{
			int16_t angleDeg = (int16_t)(theta_rel_deg());
			if(angleDeg < 0)
				angleDeg += 360;
	//		print_int32(angleDeg);
			print_int16(angleDeg);
	//		print_int16(dma_adc_raw[ADC_CHAN_BEMF_C]);

			float t = time_seconds()-tstart;

			switch (bang_state)
			{
				case ADVANCE_TO_ZERO:
				{
					if(angleDeg != 0)
					{
						theta+=.05;
					}
					else if (angleDeg == 0)
					{
						bang_state = REST_AT_ZERO;
						tstart = time_seconds();
					}
					break;
				}
				case REST_AT_ZERO:
				{
					if(t > 2)
						bang_state = ADVANCE_TO_ANGLE;
					break;
				}
				case ADVANCE_TO_ANGLE:
				{
					if(angleDeg != angle_set)
					{
						theta-=.05;
					}
					else if (angleDeg - angle_set < 1 && angleDeg-angle_set > - 1)
					{
						bang_state = REST_AT_ANGLE;
						tstart = time_seconds();
					}
					break;
				}
				case REST_AT_ANGLE:
				{
					if(t > 2)
					{
						angle_set += angle_inc;
						if(angle_set >= 360)
							angle_set = 0;
						bang_state = ADVANCE_TO_ANGLE;
					}
					break;
				}
				default:
				{
					break;
				}
			}

			Va = A*sin(theta);
			Vb = A*sin(theta + 2*M_PI/3);
			Vc = A*sin(theta + 4*M_PI/3);
			float Valpha, Vbeta;
			uint32_t tA,tB,tC;
			clarke_transform(Va,Vb,Vc,&Valpha, &Vbeta);
			svm(Valpha, Vbeta, TIM1->ARR, &tA, &tB, &tC);
	 		TIMER_UPDATE_DUTY(tA,tB,tC);

			HAL_Delay(10);
			if(TIM14_ms()>=led_ts)
			{
				//			dir = !dir&1;
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
				led_state = !led_state & 1;
				led_ts = TIM14_ms() + 200;
			}
		}
	 */

	float x1,x2;
	x1 = 0; x2 = 0;
	while(1)
	{
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);
		//		float Va_m, Vb_m, Vc_m;
		//		convert_phase_voltage(&Va_m,&Vb_m, &Vc_m);
		//		float theta_observer = observer_update(Va_m, Vb_m, i_a, i_b, &x1, &x2);
		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);
		float i_q, i_d;
		park_transform(i_alpha, i_beta, theta_rel_rad(), &i_q, &i_d);

		t = time_seconds();
		Va = A*sin(f_motor*t);
		Vb = A*sin(f_motor*t + 2*M_PI/3);
		Vc = A*sin(f_motor*t + 4*M_PI/3);

		float Valpha, Vbeta;
		uint32_t tA,tB,tC;
		clarke_transform(Va,Vb,Vc,&Valpha, &Vbeta);
		svm(Valpha, Vbeta, TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		//		int32_t val1 = (int32_t)dma_adc_raw[ADC_CHAN_CURRENT_A];
		//		int32_t val1 = (int32_t)(dma_adc_raw[ADC_CHAN_BEMF_A]);
		int32_t val1 = (int32_t)(i_q*1000.0);
//		int32_t val1 = tA;
		print_int32(val1);

		//		int32_t val2 = (int32_t)dma_adc_raw[ADC_CHAN_CURRENT_B];
		//		int32_t val2 = (int32_t)(dma_adc_raw[ADC_CHAN_BEMF_B]);
//		int32_t val2 = (int32_t)(i_b*1000.0);
//		int32_t val2 = tB;
//		print_int32(val2);

		//		int32_t val3 = (int32_t)dma_adc_raw[ADC_CHAN_CURRENT_C];
		//		int32_t val3 = (int32_t)(dma_adc_raw[ADC_CHAN_BEMF_C]);
//		int32_t val3 = (int32_t)(i_c*1000.0);
//		int32_t val3 = tC;
//		print_int32(val3);

		if(TIM14_ms()>=led_ts)
		{
			//			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = TIM14_ms() + 200;
		}
	}

}
