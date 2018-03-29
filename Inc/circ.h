/*
 * circ.h
 *
 *  Created on: Nov 20, 2017
 *      Author: Ocanath
 */

#ifndef CIRC_H_
#define CIRC_H_
#define CIRC_BUF_SIZE 3

typedef struct circularBuffer
{
	int buf[CIRC_BUF_SIZE];
}circularBuffer;

void updateCircBuf(circularBuffer * c, float newSample);
int rollingAverage_2(circularBuffer * c, int newSample);
int rollingAverage_1(circularBuffer * c, int newSample);


#endif /* CIRC_H_ */
