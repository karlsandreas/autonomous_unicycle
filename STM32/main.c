#include "main.h"
#include "regulator.c"
#include "kalman_filter.c"
#include <stdio.h>
#include <stdbool.h>

/*
float states_to_struct_regulator(float setpoint, float dt, float theta, float theta_d, float x, float x_d)
{
    States curr_states = {theta,theta_d,x,x_d};
    LookaheadSpeedRegulator(setpoint, )
}
*/

int main()
{
    States curr_states = {0.0,0.0,0.0,0.0};
    Matrix covs = {0.0,0.0,0.0,0.0};
    Matrix qs = {1.0,1.0,1.0,1.0};
    int a = 0;
    while (a <= 10) {
        pitch_kalman_filter_predict(1.0, 0.01, &curr_states , &covs, &qs);
        float torque = LookaheadSpeedRegulator(1.0, curr_states.x1, curr_states.x2, curr_states.x4, 0.01);
        printf("Torque %f , angle %f \n", torque, curr_states.x1);
        a += 1;
    }

    return 0; 
}
