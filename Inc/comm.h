/*
 * comm.h
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */

#ifndef COMM_H_
#define COMM_H_
#include "init.h"

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;

extern float m_q_offset;
extern float kd_gain;

#define NUM_PRES_UART_BYTES 	9	//number of uart bytes used for pressure data transmission
#define NUM_BYTES_UART_DMA		18	//double buffer for uart, in case we lose a byte somewhere in the transmission
#define NUM_SPI_BYTES 			14	//equal to 5+NUM_PRES_UART_BYTES. first 5 bytes reserved

extern uint8_t control_mode;

uint8_t new_uart_packet;
uint8_t new_spi_packet;

#define CMD_LED_OFF 		0
#define CMD_LED_ON 			1
#define CMD_CHANGE_PWM 		2
#define CMD_CHANGE_IQ		3
#define CMD_CHANGE_POS 		30
#define CMD_CHANGE_KD		31
#define CMD_ZERO_POS 		32


#define CMD_GET_ENCODER_POS	5	//the flux position of the rotor in ABSOLUTE frame (dh, once unwrap works maybe record offset)
#define CMD_HARD_BRAKE		6
#define CMD_DRIVER_ENABLE	7
#define CMD_DRIVER_DISABLE	8
#define CMD_ROTOR_POS		9
#define CMD_ROTOR_SPEED		10
#define CMD_RESET_T			11	//reset commutation interval estimation
#define CMD_RESET_POS		12

#define CMD_SLEEP			14
#define CMD_WAKEUP			23

#define CMD_PLAY_TONE		15	//play a tone! master data corresponds to

#define CMD_SET_FOC_MODE	16
#define CMD_SET_TRAP_MODE	17
#define CMD_SET_SIN_MODE	18

#define CMD_EN_PRES			19
#define CMD_DIS_PRES		20
#define CMD_READ_PRES		21

#define CMD_NO_ACTION		22


extern uint32_t motor_update_ts;	//time of last spi transaction, for timeout

uint16_t r_word;
uint16_t t_word;
uint8_t r_data[NUM_SPI_BYTES];
uint8_t t_data[NUM_SPI_BYTES];
uint8_t pres_data[NUM_PRES_UART_BYTES];
uint8_t uart_read_buffer[NUM_BYTES_UART_DMA];
uint8_t r_flag;
uint8_t t_flag;
int32_t gl_master_duty;
int32_t gl_prev_master_duty;

float gl_iq_u;


uint8_t sleep_flag;




void parse_master_cmd();
void execute_master_cmd();
void handle_uart_buf();

#endif /* COMM_H_ */
