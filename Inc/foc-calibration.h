/*
 * foc-calibration.h
 *
 *  Created on: Mar 20, 2019
 *      Author: Ocanath
 */

#ifndef FOC_CALIBRATION_H_
#define FOC_CALIBRATION_H_
#include "init.h"
#include "foc_commutation.h"

void manual_align_calib();
void check_encoder_region();
float check_align_offset(uint32_t run_time, float tau_ref);


void test_foc();

#endif /* FOC_CALIBRATION_H_ */
