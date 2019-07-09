/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"

float kd_gain = .5f;
float m_q_offset = 0;
float m_gear_ratio_conv = 0.134497135f;
uint32_t motor_update_ts = 0;	//time of last spi transaction, for timeout

uint8_t new_uart_packet = 0;
uint8_t new_spi_packet = 0;
uint8_t press_data_transmit_flag = 0;

uint16_t r_word = 0;
uint16_t t_word = 0;

floatsend_t rx_format;
floatsend_t tx_format;

uint8_t r_data[NUM_SPI_BYTES] = {0};
uint8_t t_data[NUM_SPI_BYTES] = {0};
uint8_t r_flag = 0;
uint8_t t_flag = 0;

int32_t gl_prev_master_duty = 0;
int32_t gl_master_duty=0;

uint8_t sleep_flag = 0;

float gl_iq_u = 0;

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
	new_spi_packet = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //Bird
{
	//press_data_transmit_flag = 1;
	new_uart_packet = 1;
}

/*
 * TODO: make this function NON BLOCKING.
 * This is not an acceptable long-term solution to
 * data alignment.
 */
void pressure_data_align(void)
{
	TIMER_UPDATE_DUTY(0,0,0);	//FOC cannot be interrupted without a full disable. this is EXTREMELY important. Disable the driver if you do ANYTHING time consuming
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
	uint32_t align_timeout = HAL_GetTick() + 1000;
	uint32_t blink_ts = 0;
	uart_read_buffer[0] = 0;
	while(HAL_GetTick() < align_timeout)	//i'll give you a full 100ms before you get back to doing what you're supposed to do, i.e. moving a motor.
	{
		HAL_UART_Receive(&huart1, uart_read_buffer, 1, 100);
		if(uart_read_buffer[0] == 's')
		{
			HAL_UART_Receive(&huart1, uart_read_buffer, NUM_BYTES_UART_DMA-1, 1000);
			break;
		}
		if(HAL_GetTick()>blink_ts)
		{
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
			blink_ts = HAL_GetTick()+50;
		}
	}
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
}


//
///*
// * TODO: use flag, take this out and do it in the main loop (to prioritize motor control).
// * why is double-buffering necessary?
// */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //Bird
//{
//	new_uart_packet = 1;
//}
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
	case CMD_CHANGE_KD:
	{
		floatsend_t kd_format;
		int i;
		for(i=0;i<4;i++)
			kd_format.d[i]=r_data[i+1];
		kd_gain = kd_format.v;
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
	case CMD_EN_PRES:
		pressure_data_align();
		press_data_transmit_flag = 1;
		break;
	case CMD_DIS_PRES:  //Bird
		press_data_transmit_flag = 0;
		break;
/* Bootloader Related Case. */
	case CMD_BOOTLOAD:
		asm("NOP");
		NVIC_SystemReset();
		break;
	case CMD_SLEEP:  //Bird
		sleep_reset();
		break;
	case CMD_WAKEUP:
		break;
	default:
		break;
	}
}



void handle_uart_buf()
{
	/*
	 * we've recieved a new round of data, so load it into the spi transmit buffer.
	 * the motor control spi protocol will carry it over to the master
	 */
/*
	int i;
	for(i = 0; i < NUM_BYTES_UART_DMA; i++)
	{
		if (uart_read_buffer[i] == 's')
		{
			int j;
			for(j = 0; j < NUM_PRES_UART_BYTES; j++)
			{
				t_data[j+5] = uart_read_buffer[ (i+j) % (NUM_BYTES_UART_DMA-1) ];						//we can do outside of the handler
			}
			break;
		}
	}
*/

	if(press_data_transmit_flag == 1)  //Bird
	{
		//first 5 bytes of r_data and t_data are RESERVED for motor control, and must not be overwritten
		for(int i = 5; i < NUM_SPI_BYTES; i++)
		{
			t_data[i] = uart_read_buffer[i-5];
		}
	}

}

