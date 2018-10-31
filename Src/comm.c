/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"


uint8_t new_spi_packet = 0;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	new_spi_packet = 1;
}

uint16_t r_word = 0;
uint16_t t_word = 0;

uint8_t r_data[NUM_SPI_BYTES] = {0};
uint8_t t_data[NUM_SPI_BYTES] = {0};
uint8_t r_flag = 0;
uint8_t t_flag = 0;

int32_t gl_prev_master_duty = 0;
int32_t gl_master_duty=0;

uint8_t sleep_flag = 0;

float gl_iq_u = 0;

/*
 * Command structure is determined by the highest 4 bits
 */


int mode = 0;
#define MODE_SEND_ROTOR_POS  	2
#define MODE_SEND_ROTOR_SPEED  	3


void parse_master_cmd()
{
	uint8_t master_cmd = r_data[0];
	int32_t master_data = r_data[1] << 24 | r_data[2] << 16 | (r_data[3] << 8) | r_data[4];

	switch(master_cmd)
	{
	case CMD_LED_OFF:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		break;
	case CMD_LED_ON:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		break;
	case CMD_SET_FOC_MODE:
		control_mode = FOC_MODE;
		break;
	case CMD_SET_TRAP_MODE:
		control_mode = TRAPEZOIDAL_MODE;
		break;
	case CMD_CHANGE_PWM :
		gl_master_duty = master_data;
		break;
	case CMD_CHANGE_IQ :
	{
		uint32_t r_word = (r_data[1]<<24) | (r_data[2] << 16) | (r_data[3] << 8) | r_data[4];
		float * tmp = (float *)(&r_word);
		gl_iq_u = *tmp;
		break;
	}
	case CMD_GET_ENCODER_POS :
		break;
	case CMD_HARD_BRAKE:
		break;
	case CMD_ROTOR_POS:
		mode = MODE_SEND_ROTOR_POS;
		break;
	case CMD_ROTOR_SPEED:
		mode = MODE_SEND_ROTOR_SPEED;
		break;
	case CMD_RESET_POS:
		gl_rotorPos = master_data;
		/*
		 *	symmetric bit depth in new protocol so sign extension not necessary, but resetting the encoder
		 *	is necessary. This is challenging, and more thought is needed.
		 *	TODO: update this after developing the hybrid control scheme
		 */
		break;
	case CMD_RESET_T:
		gl_rotorInterval = 0;
		break;
	case CMD_DRIVER_ENABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
		break;
	case CMD_DRIVER_DISABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
		break;
	case CMD_SLEEP:
		sleep_flag = 1;
		break;
	default:
		break;
	}
}


