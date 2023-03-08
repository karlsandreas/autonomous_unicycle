/*
 * kalman_filter.h
 *
 *  Created on: Mar 8, 2023
 *      Author: jonathanloov
 */

#ifndef INC_CTRL_KALMAN_FILTER_H_
#define INC_CTRL_KALMAN_FILTER_H_

void pitch_kalman_filter_predict(float input, float dt, States *s, Matrix *q);
void pitch_kalman_filter_update(float sensor, float dt, States *s, Matrix *q);



#endif /* INC_CTRL_KALMAN_FILTER_H_ */
