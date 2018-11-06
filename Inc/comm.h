/*
 * comm.h
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */

#ifndef COMM_H_
#define COMM_H_
#include "init.h"
#include "commutation.h"

#define NUM_SPI_BYTES 22

uint8_t new_spi_packet;

#define CMD_LED_OFF 		0
#define CMD_LED_ON 			1
#define CMD_CHANGE_PWM 		2
#define CMD_CHANGE_IQ		3
#define CMD_GET_ENCODER_POS	5	//the flux position of the rotor in ABSOLUTE frame (dh, once unwrap works maybe record offset)
#define CMD_HARD_BRAKE		6
#define CMD_DRIVER_ENABLE	7
#define CMD_DRIVER_DISABLE	8
#define CMD_ROTOR_POS		9
#define CMD_ROTOR_SPEED		10
#define CMD_RESET_T			11	//reset commutation interval estimation
#define CMD_RESET_POS		12
#define CMD_SLEEP			14
#define CMD_PLAY_TONE		15	//play a tone! master data corresponds to

#define CMD_SET_FOC_MODE	16
#define CMD_SET_TRAP_MODE	17
#define CMD_SET_SIN_MODE	18
#define CMD_NO_ACTION		19


uint16_t r_word;
uint16_t t_word;
uint8_t r_data[NUM_SPI_BYTES];
uint8_t t_data[NUM_SPI_BYTES];
uint8_t r_flag;
uint8_t t_flag;
int32_t gl_master_duty;
int32_t gl_prev_master_duty;

float gl_iq_u;


uint8_t sleep_flag;


control_type control_mode;

void parse_master_cmd();
void execute_master_cmd();

#endif /* COMM_H_ */
