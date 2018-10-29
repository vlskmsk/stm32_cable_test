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

typedef enum {FOC_MODE, SINUSOIDAL_MODE, TRAPEZOIDAL_MODE} control_type;


/*
 * Command structure is determined by the highest 4 bits
 */
#define CMD_SET_FOC_MODE	14
#define CMD_SET_TRAP_MODE	15
#define CMD_SET_SIN_MODE	16
#define CMD_LED_OFF 		0
#define CMD_LED_ON 			1
#define CMD_CHANGE_PWM 		2
#define CMD_CURRENT_U 		3
#define CMD_CURRENT_V 		4
#define CMD_CURRENT_W 		5
#define CMD_HARD_BRAKE		6
#define CMD_DRIVER_ENABLE	7
#define CMD_DRIVER_DISABLE	8
#define CMD_ROTOR_POS		9
#define CMD_ROTOR_SPEED		10
#define CMD_RESET_T			11	//reset commutation interval estimation
#define CMD_RESET_POS		12


uint16_t r_word;
uint16_t t_word;
#define NUM_SPI_BYTES 22
uint8_t r_data[NUM_SPI_BYTES];
uint8_t t_data[NUM_SPI_BYTES];
uint8_t r_flag;
uint8_t t_flag;
int gl_master_duty;
int gl_prev_master_duty;

control_type control_mode;

void parse_master_cmd();
void execute_master_cmd();

#endif /* COMM_H_ */
