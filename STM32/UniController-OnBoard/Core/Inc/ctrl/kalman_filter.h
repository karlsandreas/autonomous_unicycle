/*
 * kalman_filter.h
 *
 *  Created on: Mar 8, 2023
 *      Author: jonathanloov
 */

#ifndef INC_CTRL_KALMAN_FILTER_H_
#define INC_CTRL_KALMAN_FILTER_H_

#include "ctrl/common.h"

void kalman_filter_predict(float input, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs);
void kalman_filter_update(float sensor_t, float sensor_w, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs, R_error *r_vals);
void roll_kalman_filter_update(float sensor, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs, R_error *r_vals);
void roll_kalman_filter_predict(float input, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs);


#endif /* INC_CTRL_KALMAN_FILTER_H_ */
