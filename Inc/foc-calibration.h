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

typedef enum {CAL_ERR_TIMEOUT, CAL_SUCCESS} regioncheck_retv;

void manual_align_calib();

float check_align_offset(uint32_t run_time, float tau_ref);

uint8_t check_encoder_region(float start_step, uint32_t settle_time, float settle_err, float reverse_err, uint32_t timeout);
//uint8_t check_encoder_region_2(float start_offset, float diff_err_thresh, uint32_t timeout);
uint8_t check_encoder_region_2(uint32_t track_time, uint32_t timeout);

void test_foc();

#endif /* FOC_CALIBRATION_H_ */
