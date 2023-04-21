/*
 * vesc_current_reg.h
 *
 *  Created on: Mar 24, 2023
 *      Author: jonathanloov
 */

#ifndef SRC_VESC_CURRENT_REG_H_
#define SRC_VESC_CURRENT_REG_H_

// Filter frequency in Hz
#define FILTER_FREQ 0.5

typedef struct {
	float k_p;

	float setpoint;

	float input_filtered;
	float i;
} VESC_Current_Reg;

float vcr_step(VESC_Current_Reg *vcr, float dt, float current_input);

#endif /* SRC_VESC_CURRENT_REG_H_ */
