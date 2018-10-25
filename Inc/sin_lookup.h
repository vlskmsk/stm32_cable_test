/*
 * sin_lookup.h
 *
 *  Created on: Oct 19, 2018
 *      Author: Ocanath
 */

#ifndef SIN_LOOKUP_H_
#define SIN_LOOKUP_H_
#include "mag-encoder.h"

#define LOOKUP_TABLE_SIZE 256

float sin_lookup_table[LOOKUP_TABLE_SIZE];
float sin_lookup(float theta);
float cos_lookup(float theta);


#endif /* SIN_LOOKUP_H_ */
