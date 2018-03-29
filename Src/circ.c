/*
 * circ.c
 *
 *  Created on: Nov 20, 2017
 *      Author: Ocanath
 */
#include "circ.h"

void updateCircBuf(circularBuffer * c, float newSample)
{
	int i;
	for(i=CIRC_BUF_SIZE-1;i>0;i--)
		c->buf[i] = c->buf[i-1];
	c->buf[0] = newSample;
}

int rollingAverage_1(circularBuffer * c, int newSample)
{
	updateCircBuf(c,newSample);
	int i;
	int sum = 0;
	for(i=0;i<CIRC_BUF_SIZE;i++)
		sum+=c->buf[i];
	int avg = sum/(CIRC_BUF_SIZE);
	return avg;
}

int rollingAverage_2(circularBuffer * c, int newSample)
{

	int i;
	int sum = 0;
	for(i=0;i<CIRC_BUF_SIZE;i++)
		sum+=c->buf[i];
	int avg = (sum+newSample)/((CIRC_BUF_SIZE+1));
	updateCircBuf(c,avg);
	return avg;
}

