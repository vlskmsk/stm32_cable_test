/*
 * foc-calibration.c
 *
 *  Created on: Mar 20, 2019
 *      Author: Ocanath
 */
#include "foc-calibration.h"
#include "comm.h"


static float err_uart_print = 0;
static uint32_t uart_print_ts = 0;
static void uart_calib_err_print_handle()
{
	if(HAL_GetTick() > uart_print_ts)
	{
		uart_print_float(err_uart_print);
		uart_print_ts = HAL_GetTick()+10;
	}
}

/*
 * This function uses the check_align_offset function to determine whether the pre-programmed align offset is correct on a VISHAN motor
 * to within an ACCEPTABLE tolerance. If it is, it will turn on the status LED to indicate success. If not, it will power the LED off.
 */
void manual_align_calib()
{
	while(1)
	{
		float err = check_align_offset(300, 20);
		if(err < PI)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

		err_uart_print = err;	//shared for periodic printing
	}
}


/*
 * Checks a candidate align_offset (global variable, used by the encoder)
 * by producing a triangle speed waveform using constant positive and negative
 * torques; i.e. positive tau_ref for run_time ms, negative tau_ref  for run_time ms.
 * The function measures the position at the start of the waveform, the peak/tip of the
 * triangle, and the endpoint. Since the torque is constant, the speed is determined by physical
 * system (same conditions for both directions in an unloaded motor) and the quality of the align
 * offset; therefore, for a good offset, the error between the start and endpoints should be minimal.
 *
 * However, some align offsets produce ZERO speed since they're so terrible. To factor this in, the
 * midpoint is compared to the starting point; if the motor has not moved appreciably (a generous 1 radian)
 * then the align offset is no good, and an arbitrary large error is returned.
 *
 *In general, the align offset which produces the smallest error that is LESS than a given threshold (1 rad or less)
 *should be accepted.
 *
 *For motors with fixed encoder mounts, this function should be used in tandem with sweep_align.
 *for motors with variable encoder mounts (and fixed align offset values) this function should
 *relay visual feedback to a human operator, through the light and potentially a matlab plot.
 */
#define OUT_OF_BOUNDS_ERROR 1000000
volatile float gl_error_print = 0;
uint32_t print_ts=0;
float check_align_offset(uint32_t run_time, float tau_ref)
{
	uint32_t run_ts = HAL_GetTick() + run_time;
	float init_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get the initial motor position
	float prev_theta = 0;//need to track a second time for speed. unwrap smashes it
	uint8_t dir_correct_flag = 1;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta < 0)	//if speed is negative and direction is positive
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(tau_ref);

		uart_calib_err_print_handle();
	}

	/*make sure we've picked an align offset that's not completely out of bounds, by checking that we've actually traveled an appreciable distance during phase 1*/
	float mid_check_err = init_pos - prev_theta;
	//	mid_check_err &= ~0x80000000;
	if(mid_check_err < 0)
		mid_check_err = -mid_check_err;
	if(mid_check_err < 1)	//if the motor didn't move during this sweep
	{
		gl_error_print = OUT_OF_BOUNDS_ERROR;
		return gl_error_print;
	}

	run_ts = HAL_GetTick() + run_time;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta > 0)	//if speed is positive and direction is negative
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(-tau_ref);

		uart_calib_err_print_handle();
	}

	/*First, check to make sure you passed all edge condition flags*/
	//	if(dir_correct_flag == 0)
	//	{
	//		gl_error_print = OUT_OF_BOUNDS_ERROR;
	//		return gl_error_print;	//if direction mismatch, return an out of bounds error.
	//	}

	float final_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
	float err = final_pos - init_pos;
	if(err < 0)
		err = -err;
	gl_error_print = err;
	return err;
}

/*
 * Critical function for KMZ60 encoder. Determines which half of the quadrant you are actually in using the motor direction under a properly calibrated FOC.
 * Specifically, this function will try position control using FOC, with a setpoint nearby its original position. If it detects an out of bounds error
 * condition during position control, it will flip which direction is positive by changing the quadrant alignment. It does this until a stable error
 * value is achieved under FOC position control.
 */
/*original params: .4f, 50, .1f, .6f*/
uint8_t check_encoder_region(float start_step, uint32_t settle_time, float settle_err, float reverse_err, uint32_t timeout)
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t exit_ts = HAL_GetTick() + timeout;
	uint32_t ts = HAL_GetTick();
	theta_m_prev = 0;
	float theta_set = (unwrap(theta_abs_rad(), &theta_m_prev)*.5) + start_step;	//get the initial position and use it to set the motor setpoint.
	float err = 0;
	uint32_t try_ts = 100;
	uint32_t off_ts = 0;
	while(1)
	{
		handle_comms();

		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get current motor position
		err = theta_set-theta_m;		// and error
		float tau = 30*err;				//position control
		if(tau > 70)
			tau = 70;
		if(tau < -70)
			tau = -70;
		foc(tau);

		if(err < 0)
			err = -err;		//get the absolute value of the error

		if(err > reverse_err && HAL_GetTick() > ts)	//if the error is great and you haven't tried for some time, reverse the direction
		{
			foc_theta_prev -= TWO_PI;
			ts = HAL_GetTick()+try_ts;
			try_ts+=50;	//if you failed on the last attempt, try for just a liiitle bit longer. This improves stability (i.e. no infinite oscillations if you're significantly out of bounds)
		}

		if(err > settle_err)	//if the error is great,
			off_ts = HAL_GetTick();	//set the timestamp
		else
		{
			if(HAL_GetTick() - off_ts > settle_time)	//if you've settled (low error for an acceptable time) break
			{
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
				return CAL_SUCCESS;
			}
		}
		if(HAL_GetTick() > exit_ts)
			return CAL_ERR_TIMEOUT;
	}
}


static float abs_f(float in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

uint8_t check_motor_valid()
{
	float i_init[3];
	TIMER_UPDATE_DUTY(0,0,0);
	for(uint32_t run_ts = HAL_GetTick() + 10; HAL_GetTick() < run_ts; )
		conv_raw_current(&(i_init[0]),&(i_init[1]),&(i_init[2]));

	uint8_t phase_ok = 0;
	float i_m[3] = {0,0,0};
	for(int phase = 0; phase < 3; phase++)
	{
		if(phase == 0)
		{TIMER_UPDATE_DUTY(550,550,450);}
		else if (phase == 1)
		{TIMER_UPDATE_DUTY(550,450,550);}
		else if (phase == 2)
		{TIMER_UPDATE_DUTY(450,550,550);}

		for(uint32_t run_ts = HAL_GetTick() + 1; HAL_GetTick() <= run_ts; )
			conv_raw_current(&(i_m[0]),&(i_m[1]),&(i_m[2]));

		float absdiff = i_m[phase] - i_init[phase];
		if(absdiff < -0.8f)
			phase_ok |= 1 << phase;
	}
	return phase_ok;
}

void force_encoder_region()
{
	uint8_t retc = VERIFY_FAILED;
	float tau = 15.0f;
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	while(retc != VERIFY_PASSED)
	{
		retc = verify_encoder_region(tau, 20.0f, 3, 500);
		if(retc == VERIFY_FAILED)
		{
			foc_theta_prev -= TWO_PI;	//put the encoder in the correct region
		}
		if(retc == VERIFY_TIMEOUT)
		{
			if(tau > 0)
			{
				tau+=10.0f;
				if(tau > 80.0f)
					tau = 80.0f;
			}
			if(tau < 0)
			{
				tau -= 10.0f;
				if(tau < -80.0f)
					tau = -80.0f;
			}
			tau = -tau;
		}

	}
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
}


uint8_t verify_encoder_region(float tau, float diff_thresh, int pass_count_thresh, uint32_t timeout)
{
	float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
	uint32_t tau_switch_ts = HAL_GetTick();
	float pos_prev = theta_m;
	int pass_count = 0;
	while(1)
	{
		theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
		foc(tau);

		float diff = theta_m - pos_prev;
		if(abs_f(diff) >= diff_thresh)
		{
			pos_prev = theta_m;
			if((tau > 0 && diff > diff_thresh) || (tau < 0 && diff < -diff_thresh))
			{
				pass_count++;
			}
			else
			{
				pass_count--;
			}
			tau = -tau;
			tau_switch_ts = HAL_GetTick();	//TODO: instead of time, move the rotor so the change in position is at least diff_thresh
		}
		if((HAL_GetTick() - tau_switch_ts) > timeout)
			return VERIFY_TIMEOUT;

		if(pass_count >= pass_count_thresh)
			return VERIFY_PASSED;
		else if (pass_count <= -pass_count_thresh)
			return VERIFY_FAILED;

		handle_comms();

	}

}
