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
	//	MX_TIM14_Init();
	//  MX_I2C1_Init();

	//	HAL_TIM_Base_Start(&htim14);
	//	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);

	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 650;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(100);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	get_current_cal_offsets();
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	init_observer();

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	float t = 0;

	gl_angle = 0;

	TIMER_UPDATE_DUTY(0,0,0);

	obtain_encoder_offset();
	float tstart = time_seconds();


	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;
	float x1 = 0;
	float x2 = 0;

//	while(1)
//	{
//		print_int32(theta_rel_deg()*7.0);
//		HAL_Delay(1);
//	}
	while(1)
	{
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);
		//		float Va_m, Vb_m, Vc_m;
		//		convert_phase_voltage(&Va_m,&Vb_m, &Vc_m);
		//		float theta_observer = observer_update(Va_m, Vb_m, i_a, i_b, &x1, &x2);

		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		float theta_m = theta_rel_rad()*11.0;		//test this! MS motor has 22 pole pairs, which means multiply by 11
//		float theta_m = theta_rel_rad();							//work with only steven motor

		float i_q, i_d;
		park_transform(i_alpha, i_beta, theta_m, &i_q, &i_d);

		controller_PI(6.5, i_q, 0.01, 0.0000000001, &x_iq_PI, &uq);		//this sort of works
		controller_PI(0.0, i_d, 0.01, 0.0000000000, &x_id_PI, &ud);		//high current

//		const float thresh = .05;
//		if(uq > thresh)
//			uq = thresh;
//		if(uq < -thresh)
//			uq = -thresh;
//		if(ud > thresh)
//			ud = thresh;
//		if(ud < -thresh)
//			ud = -thresh;


//		uq=0.03;					//this also works with no manual spin
//		ud= 0.0;					//low current, and feedback, but technically not foc

		uint32_t tA,tB,tC;
		inverse_park_transform(uq, ud, theta_m, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		int32_t v1 = (int32_t)(i_q*10000.0);
		print_int32(v1);


		if(HAL_GetTick()>=led_ts)
		{
			//			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}

}
