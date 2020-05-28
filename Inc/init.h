/*
 * init.h
 *
 *  Created on: Mar 28, 2018
 *      Author: Ocanath
 */

#ifndef INIT_H_
#define INIT_H_
#include "main.h"
#include "stm32f3xx_hal.h"

//#define ARM_MATH_CM0
//#include <arm_math.h>
#define PSC_GEN_TIMER 7

#define COMM_PWM_PERIOD 1000

#define STAT_PORT 	GPIOF
#define STAT_PIN 	GPIO_PIN_1

#define CAL_PORT 	GPIOA
#define CAL_PIN 	GPIO_PIN_7

#define ENABLE_PORT GPIOB
#define ENABLE_PIN 	GPIO_PIN_0

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_SPI3_Init(void);
void MX_TIM1_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);




#endif /* INIT_H_ */
