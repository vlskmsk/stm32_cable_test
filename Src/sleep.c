/*
 * sleep.c
 *
 *  Created on: Apr 15, 2019
 *      Author: Ocanath
 */
#include "sleep.h"
#include "foc_commutation.h"

void config_nss_exti()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void config_nss_spi()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 *Working sleep with recovery.
 */
void sleep_reset()
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	HAL_GPIO_WritePin(ENABLE_PORT,ENABLE_PIN,0);
	TIM1->CCER = (TIM1->CCER & DIS_ALL);
	HAL_TIM_Base_Stop(&htim1);
	HAL_SuspendTick();														//If you don't suspend the tick interrupt, WFI will clear within 1ms. (not event though)
//	HAL_SPI_TransmitReceive_IT(&hspi3, t_data, r_data, NUM_SPI_BYTES);		//thought spi interrupt would work, it does not.

	HAL_PWR_EnableBkUpAccess();
	HAL_PWR_DisableSleepOnExit();

	config_nss_exti();	//exti is safer bc init is simpler? (no NVIC)
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
	config_nss_spi();	//re-enable spi

	//TODO: re-enable PA15 as nSS hardware input
	SystemClock_Config();//systemclock configuration gets screwed up by STOPMode, so recover our settings (brute force)
	HAL_ResumeTick();														//fix what you tore down
	HAL_TIM_Base_Start(&htim1);
	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;	//start_pwm();
	HAL_GPIO_WritePin(ENABLE_PORT,ENABLE_PIN,1);

}
