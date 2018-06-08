/*
 * iirSOS.c
 *
 *  Created on: May 9, 2018
 *      Author: Ocanath
 */
#include "iirSOS.h"

//int sos(iirSOS * f, int newSample)
//{
//	f->w[2] = f->w[1];
//	f->w[1] = f->w[0];
//	f->w[0] = newSample - f->a1*f->w[1] - f->a2*f->w[2];
//	return f->b0 * f->w[0] + f->b1*f->w[1] + f->b2*f->w[2];
//}
float sos_f(iirSOS * f, float newSample)
{
	f->w[2] = f->w[1];
	f->w[1] = f->w[0];
	f->w[0] = newSample - f->a1*f->w[1] - f->a2*f->w[2];
	return f->gain*(f->b0 * f->w[0] + f->b1*f->w[1] + f->b2*f->w[2]);
}
