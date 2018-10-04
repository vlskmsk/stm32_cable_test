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

volatile uint32_t t_l = 0;


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

	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);


	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 750;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

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

	gl_angle = 0;

	TIMER_UPDATE_DUTY(0,0,0);

	float x1 = 0;
	float x2 = 0;
	float Va_m, Vb_m, Vc_m;
	float theta;
	float i_a,i_b,i_c;
	float i_alpha,i_beta;
	uint32_t tA,tB,tC;

	obtain_encoder_offset();
	TIMER_UPDATE_DUTY(500,500,500);
	HAL_Delay(100);

	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;

	float iq_ref = 20;
	float id_ref = 0;
	uint32_t led_ts = 0;

	uint32_t t_p = ((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT));
	while(1)
	{

		uint32_t tc = ((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT));
		t_l = tc - t_p;
		t_p = tc;

		conv_raw_current(&i_a,&i_b, &i_c);
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		/*
		 * TODO:
		 * 1. test compensation for adc misalignment
		 * 2. test observer with improved shunts
		 * 3. see if raw observer error is characterizable/constant, try to locate source
		 */
//		convert_phase_voltage(&Va_m,&Vb_m, &Vc_m);
//		clarke_transform(Va_m,Vb_m,Vc_m, &Va_m,  &Vb_m);
//		float theta_o = observer_update(Va_m, Vb_m, i_alpha, i_beta, &x1, &x2)-M_PI;

		float theta_enc = theta_rel_rad();							//work with only steven motor
		theta = theta_enc;
//		theta = -time_seconds()*TWO_PI;

		float sin_theta,cos_theta;
		float theta_elec = theta_rel_rad();		//test this! MS motor has 22 pole pairs, which means multiply by 11
		theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

		float i_q, i_d;
		park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q, &i_d);
//
		controller_PI(iq_ref, i_q, 0.01, 0.00000001, &x_iq_PI, &uq);		//this sort of works
		controller_PI(id_ref, i_d, 0.01, 0.0000000001, &x_id_PI, &ud);		//high current
//
		inverse_park_transform(uq, ud, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		//		arm_inv_park_f32(  ud,uq,  &i_alpha,&i_beta, sin(theta_observer),cos(theta_observer));
		//		arm_cos_f32(theta_observer);

		//		inverse_clarke_transform(i_alpha,i_beta,&Va_m,&Vb_m,&Vc_m);
		//		tA = (uint32_t)(Va_m*1000+500);	tB = (uint32_t)(Vb_m*1000+500);	tC = (uint32_t)(Vc_m*1000+500);
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}
}
