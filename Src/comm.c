/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"

uint32_t motor_update_ts = 0;	//time of last spi transaction, for timeout

uint8_t new_uart_packet = 0;
uint8_t new_spi_packet = 0;
uint8_t press_data_transmit_flag = 0;



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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)  //Bird
{
	new_spi_packet = 1;
}

/*
 * TODO: use flag, take this out and do it in the main loop (to prioritize motor control).
 * why is double-buffering necessary?
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //Bird
{
	new_uart_packet = 1;
//	asm("NOP");																//why the NOP
}

void parse_master_cmd()
{
	uint8_t master_cmd = r_data[0];
	int32_t master_data = (r_data[1]<<24) | (r_data[2] << 16) | (r_data[3] << 8) | r_data[4];

	switch(master_cmd)
	{
	case CMD_LED_OFF:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		break;
	case CMD_LED_ON:
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		break;
	case CMD_SET_FOC_MODE:

		break;
	case CMD_SET_TRAP_MODE:

		break;
	case CMD_CHANGE_PWM :
		gl_master_duty = master_data;
		break;
	case CMD_CHANGE_IQ :
	{
//		uint32_t r_word = (r_data[1]<<24) | (r_data[2] << 16) | (r_data[3] << 8) | r_data[4];
//		float * tmp = (float *)(&(r_data[1]));
//		gl_iq_u = *tmp;
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
		/*
		 *	symmetric bit depth in new protocol so sign extension not necessary, but resetting the encoder
		 *	is necessary. This is challenging, and more thought is needed.
		 *	TODO: update this after developing the hybrid control scheme
		 */
		break;
	case CMD_RESET_T:
		break;
	case CMD_DRIVER_ENABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
		break;
	case CMD_DRIVER_DISABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
		break;
	case CMD_EN_PRES:  //Bird
		asm("NOP");						//why?
		HAL_UART_DMAResume(&huart1);
		press_data_transmit_flag = 1;
		break;
	case CMD_DIS_PRES:  //Bird
		asm("NOP");
		HAL_UART_DMAPause(&huart1);
		press_data_transmit_flag = 0;
		break;
	case CMD_SLEEP:  //Bird
		sleep_flag = 1;
		break;
	case CMD_WAKEUP:
		sleep_flag = 0;
		break;
	default:
		break;
	}
}



void handle_uart_buf()
{
	/*
	 * once we've recieved a flag that new uart data must be parsed, scan the double buffer for a complete data set
	 */
	int i;
	for(i = 0; i < NUM_BYTES_UART_DMA; i++)
	{
		if (uart_read_buffer[i] == 's')
		{
			int j;
			for(j = 0; j < NUM_PRES_UART_BYTES; j++)
			{
				pres_data[j] = uart_read_buffer[ (i+j) % (NUM_BYTES_UART_DMA-1) ];						//we can do outside of the handler
			}
			break;
		}
	}

	/*
	 * we've recieved a new round of data, so load it into the spi transmit buffer.
	 * the motor control spi protocol will carry it over to the master
	 */
	if(press_data_transmit_flag == 1)  //Bird
	{
		//first 5 bytes of r_data and t_data are RESERVED for motor control, and must not be overwritten
		for(int i = 5; i < NUM_SPI_BYTES; i++)
		{
			t_data[i] = pres_data[i-5];
		}
	}
}

