#include <math.h>
#include "main.h"
#include <stdio.h>


float cov11_angle = 10.0;
float cov12_angle = 0.0;
float cov21_angle = 10.0;
float cov22_angle = 0.0;

float cov33_wheel = 100.0;
float cov34_wheel = 0.0;
float cov43_wheel = 0.0;
float cov44_wheel = 100.0;

void kalman_filter_predict(float input, float dt, States *s, Matrix *q_t, Matrix *q_w)
{
    float X11 = s->x1 + dt * s->x2;
    //float X21 = s->x2;
    float X31 = s->x3 + dt * s->x4;
    //float X41 = s->x4; 

    //printf("Us: %0.10f %0.10f\n", U1, U2);
    s->x1 = X11;
    //s->x2 = X21;
    s->x3 = X31;
    //s->X4 = X41; 

    /* Dot product covariance and state transistion model*/
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
    float c11 = cov11_angle + (dt * cov21_angle);
    float c12 = cov12_angle + (dt * cov22_angle);
    float c21 = cov21_angle;
    float c22 = cov22_angle;

    float c33 = cov33_wheel + (dt * cov43_wheel);
    float c34 = cov34_wheel + (dt * cov44_wheel);
    float c43 = cov43_wheel;
    float c44 = cov44_wheel;

    //printf("Cs: %0.15f %0.15f %0.15f %0.15f\n", c11, c12, c21, c22);
    /* c matrix transposed with state transistion model
    Fm11 Fm12    Fm11 Fm21
    Fm21 Fm22 => Fm12 Fm22
    */
    float p11 = c11 + c12 * dt;
    float p12 = c12;
    float p21 = c21 + c22;
    float p22 = c22;

    float p33 = c33 + c34 * dt;
    float p34 = c34;
    float p43 = c43 + c44 * dt;
    float p44 = c44;

    //printf("Ps: %0.15f %0.15f %0.15f %0.15f \n", p11, p12, p21, p22);
 
    cov11_angle = p11 + q_t->m11;  
    cov12_angle = p12 + q_t->m12;
    cov21_angle = p21 + q_t->m21;
    cov22_angle = p22 + q_t->m22;

    cov33_wheel = p33 + q_w->m11;  
    cov34_wheel = p34 + q_w->m12;
    cov43_wheel = p43 + q_w->m21;
    cov44_wheel = p44 + q_w->m22;
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
}


void pitch_kalman_filter_predict(float input, float dt, States *s, Matrix *q)
{
    /* input is acceleration input*/

    /* Control matrix times control input, acceleration*/
    float U1 = 0.5*pow(dt,2) * input * SENSOR_POS_Z;
    float U2 = dt * input * SENSOR_POS_Z;
    //printf("Input: %f\n", input);
    float F12 = dt;

    float X11 = s->x1 + F12 * s->x2;
    float X21 = s->x2;
    //printf("Us: %0.10f %0.10f\n", U1, U2);
    s->x1 = X11 + U1;
    s->x2 = X21 + U2;
    /*
    c matrix = cov11_angle cov12_angle
               cov21_angle cov22_angle
    */

    /* Dot product covariance and state transistion model*/
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
    float c11 = cov11_angle + (F12 * cov21_angle);
    float c12 = cov12_angle + (F12 * cov22_angle);
    float c21 = cov21_angle;
    float c22 = cov22_angle;
    //printf("Cs: %0.15f %0.15f %0.15f %0.15f\n", c11, c12, c21, c22);
    /* c matrix transposed with state transistion model
    Fm11 Fm12    Fm11 Fm21
    Fm21 Fm22 => Fm12 Fm22
    */
    float p11 = c11 * F11 + c12 * F12;
    float p12 = c11 * F21 + c12 * F22;
    float p21 = c21 * F11 + c22 * F12;
    float p22 = c22 * F22;
    //printf("Ps: %0.15f %0.15f %0.15f %0.15f \n", p11, p12, p21, p22);
 
    cov11_angle = p11 + q->m11;  
    cov12_angle = p12 + q->m12;
    cov21_angle = p21 + q->m21;
    cov22_angle = p22 + q->m22;
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
}

float r_angle = 0.3; //Measurement error
float r_wheel = 20;  //Measurement error

void kalman_filter_update(float sensor_t, float sensor_w, float dt, States *s, Matrix *q_t, Matrix *q_w)
{   
    //Sensor_t is top angluar velocity, senors_w is wheel velocity
    /* sensor is angular velocity */
    float diff_t = sensor_t - s->x2;
    float diff_w = sensor_w - ((1/WHEEL_RAD)*RPM_TO_RADS*s->x4);

    //printf("Diff: %f\n", diff);

    /* Gain calculation */
    float gain1_t = cov12_angle * (1 / (cov22_angle + r_angle));
    float gain2_t = cov22_angle * (1 / (cov22_angle + r_angle));
    float k = WHEEL_RAD * RPM_TO_RADS;
    float gain1_w = (cov34_wheel * k) / (cov44_wheel * pow(K,2) + r_wheel);
    float gain2_w = (cov44_wheel * k) / (cov44_wheel * pow(K,2) + r_wheel);


    //printf("Gain1: %f Gain2: %f\n", gain1, gain2);
    /* Update states */
    s->x1 = s->x1 + gain1_t * diff_t;
    s->x2 = s->x2 + gain2_t * diff_t;
    s->x3 = s->x3 + gain1_w * diff_w;
    s->x4 = s->x4 + gain2_w * diff_w;

    //printf("New states: x1 = %f x2 = %f\n", s->x1, s->x2);

    /* Covariance update */
    //printf("Update covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
    float c11 = (-cov12_angle*cov21_angle + cov11_angle*(cov22_angle+r_angle))/(cov22_angle + r_angle);
    float c12 = (cov12_angle*r_angle)/(cov22_angle + r_angle);
    float c21 = (cov21_angle*r_angle)/(cov22_angle + r_angle);
    float c22 = (cov22_angle*r_angle)/(cov22_angle + r_angle);

    cov11_angle = c11;
    cov12_angle = c12;
    cov21_angle = c21;
    cov22_angle = c22;

    float c33 = cov33_wheel + cov44_wheel*pow(gain1_w,2)*pow(k,2) - cov34_wheel*gain1_w*k - cov43_wheel*gain1_w*k + pow(gain1_w,2)*r_wheel;
    float c34 = cov34_wheel + cov44_wheel*gain1_w*gain2_w*pow(k,2) - cov44_wheel*gain1_w*k - cov34_wheel*gain2_w*k + gain1_w*gain2_w*r_wheel;
    float c43 = cov43_wheel + cov44_wheel*gain1_w*gain2_w*pow(k,2) - cov44_wheel*gain1_w*k - cov43_wheel*gain2_w*k + gain1_w*gain2_w*r_wheel;
    float c44 = cov44_wheel + cov44_wheel*pow(gain2_w,2)*pow(k,2) - 2*cov44_wheel*gain2_w*k + pow(gain2_w,2)*r_wheel;

    cov33_wheel = c33;
    cov34_wheel = c34;
    cov43_wheel = c43;
    cov44_wheel = c44;
    
 

    //printf("Update after covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);

}

void pitch_kalman_filter_update(float sensor, float dt, States *s, Matrix *q)
{   
    //Sensor_t is top angluar velocity, senors_w is wheel velocity
    /* sensor is angular velocity */
    float diff = sensor - s->x2;
    //printf("Diff: %f\n", diff);

    /* Gain calculation */
    float gain1 = cov12_angle * (1 / (cov22_angle + r_angle));
    float gain2 = cov22_angle * (1 / (cov22_angle + r_angle));
    //printf("Gain1: %f Gain2: %f\n", gain1, gain2);
    /* Update states */
    s->x1 = s->x1 + gain1 * diff;
    s->x2 = s->x2 + gain2 * diff;
    //printf("New states: x1 = %f x2 = %f\n", s->x1, s->x2);

    /* Covariance update */
    //printf("Update covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);
    float c11 = (-cov12_angle*cov21_angle + cov11_angle*(cov22_angle+r_angle))/(cov22_angle + r_angle);
    float c12 = (cov12_angle*r_angle)/(cov22_angle + r_angle);
    float c21 = (cov21_angle*r_angle)/(cov22_angle + r_angle);
    float c22 = (cov22_angle*r_angle)/(cov22_angle + r_angle);

    cov11_angle = c11;
    cov12_angle = c12;
    cov21_angle = c21;
    cov22_angle = c22;
    //printf("Update after covs: %0.15f %0.15f %0.15f %0.15f\n", cov11_angle, cov12_angle, cov21_angle, cov22_angle);

}

float x_d = 0.0;
float x_d_d = 0.00;
float cov_vel_11 = 10.0; //P
float cov_vel_12 = 0.0;
float cov_vel_22 = 10.0; //P
float cov_vel_21 = 0.0;


float wheel_velocity_kalman_filter_predict(float dt)
{   
    float q_factor = 10.0;
    float q11 = q_factor * pow(dt,2);
    float q12 = q_factor * dt;
    float q21 = q_factor * dt;
    float q22 = q_factor;
    
    float cur_x_d = x_d + x_d_d * dt;
    //x_d_d = x_d - cur_x_d;
    //x_d = cur_x_d;

    /* Dot product covariance and state transistion model*/
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov_vel_11, cov_vel_12, cov_vel_21, cov_vel_22);
    float c11 = cov_vel_11 + dt*(cov_vel_22*dt + cov_vel_12 + cov_vel_21);
    float c12 = cov_vel_12 + cov_vel_22*dt;
    float c21 = cov_vel_21 + cov_vel_22*dt;
    float c22 = cov_vel_22;
    //printf("Cs: %0.15f %0.15f %0.15f %0.15f\n", c11, c12, c21, c22);
 
    cov_vel_11 = c11 + q11;  
    cov_vel_12 = c12 + q12;
    cov_vel_21 = c21 + q21;
    cov_vel_22 = c22 + q22;

    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov_vel_11, cov_vel_12, cov_vel_21, cov_vel_22);
    return cur_x_d;
}


void wheel_velocity_kalman_filter_update(float sensor, float dt)
{
    /* sensor is velocity */
    float diff = sensor - x_d;
    //printf("Diff: %f\n", diff);

    /* Gain calculation */
    float gain1 = cov_vel_11 / (cov_vel_11 + E_WHEEL);
    float gain2 = cov_vel_21 / (cov_vel_11 + E_WHEEL);
    //printf("Gain1: %f Gain2: %f\n", gain1, gain2);
    /* Update states */
    float last_x_d = x_d;
    x_d = x_d + gain1 * diff;
    x_d_d = x_d_d + gain2 * diff;

    //printf("New states: x1 = %f x2 = %f\n", x_d , x_d_d);

    /* Covariance update */
    //printf("Update covs: %0.15f %0.15f %0.15f %0.15f\n", cov_vel_11, cov_vel_12, cov_vel_21, cov_vel_22);
    float A = 1-gain1;
    
    float c11 = cov_vel_11*pow(A,2);
    float c12 = A*(cov_vel_11+cov_vel_12)*(-gain2);
    float c21 = A*(-gain2*cov_vel_11 + cov_vel_21);
    float c22 = -gain2 * (-gain2*cov_vel_11 + cov_vel_21) + (-gain2*cov_vel_21 + cov_vel_22);

    cov_vel_11 = c11;
    cov_vel_12 = c12;
    cov_vel_21 = c21;
    cov_vel_22 = c22;
    //printf("Update after covs: %0.15f %0.15f %0.15f %0.15f\n", cov_vel_11, cov_vel_12, cov_vel_21, cov_vel_22);
}

float cov_vel = 1.0;

float simple_wheel_filter(float sensor, float dt)
{
    float q_vel = 10 * dt;
    float gain = cov_vel/(cov_vel + E_WHEEL);

    cov_vel = (1-gain)*cov_vel + q_vel ;
    
    float diff = sensor - x_d;
    float cur_x_d = x_d + gain*diff;
    //printf("Gain = %f x_d = %f diff = %f \n", gain, cur_x_d, diff);
    x_d = cur_x_d;
    return cur_x_d;
}