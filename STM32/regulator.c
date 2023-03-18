#include <math.h>
#include "regulator.h"
#include <stdbool.h>
//#include <stdio.h>

float stop_time_x_d(float setpoint_x_d, float theta, float theta_d, float x_d)
{
    float x_d_diff = abs(x_d - setpoint_x_d);
    float out = x_d_diff * 0.5 + 1.2;
    //printf("Stop time x_d: %f\n", out);
    return out;
}

float LookaheadSpeedRegulator(float setpoint_x_d, float theta, float theta_d, float x_d, float dt)
{
    //printf("Setpoint: %f\n Top angle: %f\n Top angle_d %f\n Velocity %f\n Delta time %f\n", setpoint_x_d, theta, theta_d, x_d, dt);
    float t_theta = 2.0;
    float t_x_d = stop_time_x_d(setpoint_x_d, theta, theta_d, x_d);

    float delta_tau_theta = 1 / (pow(t_theta,2) / 2 * param_D) * (0 - theta - theta_d * t_theta);
    //printf("delta_tau_theta: %f\n", delta_tau_theta);
    float delta_tau_theta_d = 1 / (t_theta * param_D) * -theta_d;
    //printf("delta_tau_theta_d: %f\n", delta_tau_theta_d);
    float delta_tau_x_d = 1 / (t_x_d * param_B + pow(t_x_d, 3) / 6 * param_E * param_D) * (setpoint_x_d - x_d - t_x_d * param_E * theta - pow(t_x_d, 2) / 2 * param_E * theta_d);  
    //printf("delta_tau_x_d: %f\n", delta_tau_x_d);
    float last_delta_tau = delta_tau_theta + delta_tau_theta_d + delta_tau_x_d;
    //printf("last_delta_tau: %f\n", last_delta_tau);
    float tau = - param_C / param_D * theta + last_delta_tau;
    //printf("tau: %f\n",tau);
    return tau;
}










float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
float lasterror = 0.0;
float i_error = 0.0;

void init_pid_regulator(float set_kp, float set_ki, float set_kd)
{
    kp = set_kp;
    ki = set_ki;
    kd = set_kd;
}


float pid_regulator(float setpoint, float signal, float dt)
{
    float error = setpoint - signal;

    i_error += i_error * dt;
    float d_error = (error - lasterror) / dt;

    lasterror = error;

    float out_signal = kp * error + ki * i_error + kd * d_error;

    return out_signal;
}

bool check_angle_limit(float angle)
{
    if ((angle > 0.52) || (angle < -0.52)) //30 degrees
    {
        return false;
    }
    else 
        return true;
}   