/*
 * init.h
 *
 *  Created on: Mar 28, 2018
 *      Author: Ocanath
 */

#ifndef INIT_H_
#define INIT_H_
#include "main.h"
#include "stm32f0xx_hal.h"

//#define ARM_MATH_CM0
//#include <arm_math.h>
#define PSC_GEN_TIMER 7

#define COMM_PWM_PERIOD 1000

#define STAT_PORT GPIOC
#define STAT_PIN GPIO_PIN_13

#define CAL_PORT GPIOB
#define CAL_PIN GPIO_PIN_8

#define ENABLE_PORT GPIOA
#define ENABLE_PIN GPIO_PIN_11


ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init_FOC(void);
void MX_ADC_Init_TRAP(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM1_Init(void);
void MX_TIM14_Init(void);
void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif /* INIT_H_ */
