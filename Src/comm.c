/*
 * comm.c
 *
 *  Created on: Jan 17, 2018
 *      Author: Ocanath
 */
#include "comm.h"

uint16_t r_word = 0;
uint16_t t_word = 0;

uint8_t r_data[2] = {0};
uint8_t t_data[2] = {0};
uint8_t r_flag = 0;
uint8_t t_flag = 0;
int gl_prev_master_duty = 0;
int gl_master_duty=0;


/*
 * Command structure is determined by the highest 4 bits
 */

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


int mode = 0;
#define MODE_SEND_ROTOR_POS  	2
#define MODE_SEND_ROTOR_SPEED  	3



void parse_master_cmd()
{
	int master_cmd = ((r_data[1]&0xF0)>>4);
	int master_data = ((r_data[1]&0x0F)<<8) | (r_data[0]&0xFF);
//	int master_cmd = r_word & 0xF000>>12;
//	int master_data = r_word & 0x0FFF;
	switch(master_cmd)
	{
	case CMD_LED_OFF :
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		break;
	case CMD_LED_ON :
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		break;
	case CMD_CHANGE_PWM :
		gl_master_duty = master_data;
		break;
	case CMD_CURRENT_U :
		break;
	case CMD_CURRENT_V :
		break;
	case CMD_CURRENT_W :
		break;
	case CMD_HARD_BRAKE:
		break;
	case CMD_ROTOR_POS:
		mode = MODE_SEND_ROTOR_POS;
		break;
	case CMD_ROTOR_SPEED:
		mode = MODE_SEND_ROTOR_SPEED;
		break;
	case CMD_DRIVER_ENABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
		break;
	case CMD_DRIVER_DISABLE:
		HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
		break;
	case CMD_RESET_POS:
		gl_rotorPos = 0;
		break;
	case CMD_RESET_T:
		gl_rotorInterval = 0;
		break;
	default:
		break;
	}
}

void execute_master_cmd()
{
	switch(mode)
	{
	case MODE_SEND_ROTOR_POS:
		t_data[1] = (gl_rotorPos&0xFF00)>>8;
		t_data[0] = (gl_rotorPos&0x00FF);
		break;
	case MODE_SEND_ROTOR_SPEED:
		t_data[1] = (gl_rotorInterval&0xFF00)>>8;
		t_data[0] = (gl_rotorInterval&0x00FF);
		break;
	default:
		break;
	}
}

