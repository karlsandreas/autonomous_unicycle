/*
 * vesc_current_reg.c
 *
 *  Created on: Mar 24, 2023
 *      Author: jonathanloov
 */


#include "vesc_current_reg.h"

float vcr_step(VESC_Current_Reg *vcr, float dt, float current_input) {
	float tau = 1 / (FILTER_FREQ * 6.28);
	float alpha = dt / tau;

	vcr->input_filtered = vcr->input_filtered * (1 - alpha) + alpha * current_input;

	float error = vcr->setpoint - vcr->input_filtered;

	return vcr->setpoint + vcr->k_p * error;
}
