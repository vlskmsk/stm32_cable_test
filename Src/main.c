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
	//	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();

	MX_TIM14_Init();
	//	MX_USART1_UART_Init();


	start_pwm();
	TIMER_UPDATE_DUTY(500,500,500);

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);


	//	int i;
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	gl_current_input_offset = (dma_adc_raw[ADC_CHAN_CURRENT_A]+dma_adc_raw[ADC_CHAN_CURRENT_B]+dma_adc_raw[ADC_CHAN_CURRENT_C])/3;

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	gl_zero_cross_point = initZeroCrossPoint(dma_adc_raw);

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	float t = 0;
	int t_ts = HAL_GetTick();
	gl_angle = 0;
	float f_motor = 2*PI*3;
	float Va,Vb,Vc;
	float A = .2;
	int dir = 1;
	while(1)
	{
		t = (float)(HAL_GetTick() - t_ts)/1000.0;
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

		if(HAL_GetTick()>=led_ts)
		{
			dir = !dir&1;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 6000;
		}
	}



}
