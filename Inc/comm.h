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

uint16_t r_word;
uint16_t t_word;

uint8_t r_data[2];
uint8_t t_data[2];
uint8_t r_flag;
uint8_t t_flag;
int gl_master_duty;
int gl_prev_master_duty;

void parse_master_cmd();
void execute_master_cmd();

#endif /* COMM_H_ */
