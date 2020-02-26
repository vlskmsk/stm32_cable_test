/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"

float iq_limit = 80.0f;
float kd_gain = .5f;
float m_q_offset = 0;
float m_gear_ratio_conv = 0.134497135f;
uint32_t motor_update_ts = 0;	//time of last spi transaction, for timeout

uint8_t new_uart_packet = 0;
uint8_t new_spi_packet = 0;

floatsend_t rx_format;
floatsend_t tx_format;
floatsend_t mcur_format;

uint8_t r_data[MAX_SPI_BYTES] = {0};
uint8_t t_data[MAX_SPI_BYTES] = {0,0,0,0,0, 0xBD,'m','o','t','o','r','f','i','n','g','e','r'};

uint8_t uart_read_buffer[MAX_UART_BYTES] = {0};

uint8_t enable_pressure_flag = 0;

int num_uart_bytes = BARO_SENSE_SIZE;

static int gl_num_cursense_bytes = 0;

void uart_print_float(float v)
{
	floatsend_t ft;
	ft.v = v;
	HAL_UART_Transmit(&huart1, ft.d, 4, 10);
}

/*
 * Command structure is determined by the highest 4 bits
 */
int mode = 0;
#define MODE_SEND_ROTOR_POS  	2
#define MODE_SEND_ROTOR_SPEED  	3

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)  //Bird
{
	for(int i=0;i<4;i++)
		t_data[i+1] = tx_format.d[i];	//
	for(int i=0; i < gl_num_cursense_bytes; i++)
		t_data[i+5] = mcur_format.d[i];
	for(int i=0;i<4;i++)
		rx_format.d[i] = r_data[i+1];
	new_spi_packet = 1;
}

uint32_t uart_it_ts = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //Bird
{
	uart_it_ts = HAL_GetTick()+4;
	new_uart_packet = 1;
}

uint8_t control_mode = CMD_CHANGE_IQ;
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
		control_mode = CMD_CHANGE_IQ;
		break;
	}
	case CMD_CHANGE_POS:
	{
		control_mode = CMD_CHANGE_POS;
		break;
	}
	case CMD_PLAY_TONE:
	{
		if(t_data[0] == MAIN_LOOP_READY)
		{
			control_mode = CMD_PLAY_TONE;
			TIM1->PSC = (r_data[1] << 8) | r_data[2];
			TIMER_UPDATE_DUTY(490,510,490);
		}
		break;
	}
	case CMD_STOP_TONE:
	{
		TIMER_UPDATE_DUTY(500,500,500);
		control_mode = CMD_STOP_TONE;
		TIM1->PSC = 0;
		break;
	}
	case CMD_FORCE_ENCODER:
	{
		control_mode = CMD_FORCE_ENCODER;
		break;
	}
	case CMD_CHANGE_KD:
	{
		floatsend_t kd_format;
		int i;
		for(i=0;i<4;i++)
			kd_format.d[i]=r_data[i+1];
		kd_gain = kd_format.v;
		break;
	}
	case CMD_CHANGE_IQ_LIM:
	{
		floatsend_t fmt;
		for(int i = 0; i < 4; i++)
			fmt.d[i] = r_data[i+1];
		iq_limit = fmt.v;
		break;
	}
	case CMD_CHANGE_GEAR_CONV:
	{
		floatsend_t gr_format;
		int i;
		for(i=0;i<4;i++)
			gr_format.d[i] = r_data[i+1];
		m_gear_ratio_conv = gr_format.v;
		break;
	}
	case CMD_ZERO_POS:
	{
//		floatsend_t pos_format;
//		int i;
//		for(i=0;i<4;i++)
//			pos_format.d[i] = r_data[i+1];
//		m_q_offset = pos_format.v;
		m_q_offset = unwrap(theta_abs_rad(), &mech_theta_prev);
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
/* Pressure Sensor Related Case. */
	case CMD_EN_PRES_BARO:
		enable_pressure_flag = 1;
		gl_num_cursense_bytes = 0;
		num_uart_bytes = BARO_SENSE_SIZE;
		break;
	case CMD_EN_PRES_MAGSENSE:
		enable_pressure_flag = 1;
		gl_num_cursense_bytes = 0;
		num_uart_bytes = MAG_SENSE_SIZE;
		break;
	case CMD_DIS_PRES:  //Bird
		enable_pressure_flag = 0;
		break;
	case CMD_EN_CURSENSE:
		gl_num_cursense_bytes = 4;
		enable_pressure_flag = 0;
		break;
/* Bootloader Related Case. */
	case CMD_BOOTLOAD:
		asm("NOP");
		NVIC_SystemReset();
		break;
	case CMD_SLEEP:  //Bird
//		sleep_reset();
		break;
	case CMD_WAKEUP:
		break;
	default:
		break;
	}
}



