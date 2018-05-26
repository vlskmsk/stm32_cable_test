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

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);


	//	int i;
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(10);
	gl_current_input_offset = (dma_adc_raw[ADC_CHAN_CURRENT_A]+dma_adc_raw[ADC_CHAN_CURRENT_B]+dma_adc_raw[ADC_CHAN_CURRENT_C])/3;

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	gl_zero_cross_point = initZeroCrossPoint(dma_adc_raw);


	init_observer();

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	float t = 0;
	int t_ts = TIM14_ms();
	//	int t_ts = HAL_GetTick();
	gl_angle = 0;
	float f_motor = 2*PI*40;
	float Va,Vb,Vc;
	float A = .25;
	int dir = 0;



//	int i;
//	for(i=0;i<10;i++)
//	{
//		TIMER_UPDATE_DUTY(1000,1000,1000);
//		TIM14->CNT = 0;
//		while(TIM14->CNT < 50);
//		TIMER_UPDATE_DUTY(0,0,0);
//		TIM14->CNT = 0;
//		while(TIM14->CNT < 50);
//	}
//	while(1);

	//for vishan, 1/e of max value is hit in approximately 2 microseconds
	//
	float theta = 0;
	float x1,x2;
	while(1)
	{
		float i_a,i_b,i_c , Va_m, Vb_m, Vc_m;
		conv_raw_current(&i_a,&i_b, &i_c);
		convert_phase_voltage(&Va_m,&Vb_m, &Vc_m);

		float Valpha_m, Vbeta_m, Ialpha_m, Ibeta_m;
		clarke_transform(Va_m,Vb_m,Vc_m,&Valpha_m, &Vbeta_m);
		clarke_transform(i_a,i_b,i_c,&Ialpha_m, &Ibeta_m);

		observer_update(Valpha_m, Vbeta_m, Ialpha_m, Ibeta_m, &x1, &x2);

		t = (float)((TIM14_ms()*CONST_MS_TO_TICK+TIM14->CNT) - t_ts)*seconds_per_tick;
		f_motor = 3;
//		theta = cos(t*f_motor)*M_PI/2*.5*21.3*20;
		theta = cos(t*f_motor)*M_PI*10*8;

		Va = A*sin(theta);
		Vb = A*sin(theta + 2*M_PI/3);
		Vc = A*sin(theta + 4*M_PI/3);

		float Valpha, Vbeta;
		uint32_t tA,tB,tC;
		clarke_transform(Va,Vb,Vc,&Valpha, &Vbeta);
		svm(Valpha, Vbeta, TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		//		TIMER_UPDATE_DUTY(1000,1000,1000);
		if(TIM14_ms()>=led_ts)
		{
			//			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = TIM14_ms() + 200;
		}
	}



	while(1)
	{
		//		f_motor = 20*sin(.5*t)+20;
		t = (float)((TIM14_ms()*1000+TIM14->CNT) - t_ts)*.000001;
		//		t = (float)(HAL_GetTick()- t_ts)/1000;
		if(dir == 1)
		{
			Va = A*sin(t*f_motor);
			Vb = A*sin(t*f_motor + 2*M_PI/3);
			Vc = A*sin(t*f_motor + 4*M_PI/3);
		}
		else
		{
			Vb = A*sin(t*f_motor);
			Va = A*sin(t*f_motor + 2*M_PI/3);
			Vc = A*sin(t*f_motor + 4*M_PI/3);
		}

		float Valpha, Vbeta;
		clarke_transform(Va,Vb,Vc,&Valpha, &Vbeta);
		uint32_t sector;
		uint32_t tA,tB,tC;
		sector = svm(Valpha, Vbeta, TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		if(TIM14_ms()>=led_ts)
		{
			//			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = TIM14_ms() + 200;
		}
		//		float i_a,i_b,i_c;
		//		conv_raw_current(&i_a,&i_b, &i_c);
	}
}
