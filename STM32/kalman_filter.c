#include "main.h"
#include <stdio.h>
#include <math.h>


struct covariances
{   
    Matrix pitch;
    Matrix roll;
    Matrix wheel
     
};
typedef struct covariances Covariances;

covs = (Covariances) {.pitch.m11 = 10.0, .pitch.m12 = 0.0, .pitch.m21 = 10.0, .pitch.m22 = 0.0,
                      .roll.m11 = 10.0, .roll.m12 = 0.0, .roll.m21 = 10.0, .roll.m22 = 0.0,
                      .wheel.m11 = 10.0, .wheel.m12 = 0.0, .wheel.m21 = 0.0, .wheel.m22 = 10.0};


struct r_error  
{
    float pitch;
    float roll;
    float wheel;
};

typedef struct r_error R_error;

//Measurement error
r_vals = (R_error) {.pitch = 0.3, .roll = 0.3, .wheel = 20};



void kalman_filter_predict(float input, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs)
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
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", covs->pitch.m11, covs->pitch.m12, covs->pitch.m21, covs->pitch.m22);
    float c11 = covs->pitch.m11 + (dt * covs->pitch.m21);
    float c12 = covs->pitch.m12 + (dt * covs->pitch.m22);
    float c21 = covs->pitch.m21;
    float c22 = covs->pitch.m22;

    float c33 = covs->wheel.m11 + (dt * covs->wheel.m21);
    float c34 = covs->wheel.m12 + (dt * covs->wheel.m22);
    float c43 = covs->wheel.m21;
    float c44 = covs->wheel.m22;

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
 
    covs->pitch.m11 = p11 + q_t->m11;  
    covs->pitch.m12 = p12 + q_t->m12;
    covs->pitch.m21 = p21 + q_t->m21;
    covs->pitch.m22 = p22 + q_t->m22;

    covs->wheel.m11 = p33 + q_w->m11;  
    covs->wheel.m12 = p34 + q_w->m12;
    covs->wheel.m21 = p43 + q_w->m21;
    covs->wheel.m22 = p44 + q_w->m22;
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", covs->pitch.m11, covs->pitch.m12, covs->pitch.m21, covs->pitch.m22);
}




void kalman_filter_update(float sensor_t, float sensor_w, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs, R_error *r_vals)
{   
    //Sensor_t is top angluar velocity, senors_w is wheel velocity
    /* sensor is angular velocity */
    float diff_t = sensor_t - s->x2;
    float diff_w = sensor_w - ((1/WHEEL_RAD)*RPM_TO_RADS*s->x4);

    //printf("Diff: %f\n", diff);

    /* Gain calculation */
    float gain1_t = covs->pitch.m12 * (1 / (covs->pitch.m22 + r_vals->pitch));
    float gain2_t = covs->pitch.m22 * (1 / (covs->pitch.m22 + r_vals->pitch));
    float k = WHEEL_RAD * RPM_TO_RADS;
    float gain1_w = (covs->wheel.m12 * k) / (covs->wheel.m22 * pow(k,2) + r_vals->wheel);
    float gain2_w = (covs->wheel.m22 * k) / (covs->wheel.m22 * pow(k,2) + r_vals->wheel);


    //printf("Gain1: %f Gain2: %f\n", gain1, gain2);
    /* Update states */
    s->x1 = s->x1 + gain1_t * diff_t;
    s->x2 = s->x2 + gain2_t * diff_t;
    s->x3 = s->x3 + gain1_w * diff_w;
    s->x4 = s->x4 + gain2_w * diff_w;

    //printf("New states: x1 = %f x2 = %f\n", s->x1, s->x2);

    /* Covariance update */
    //printf("Update covs: %0.15f %0.15f %0.15f %0.15f\n", covs->pitch.m11, covs->pitch.m12, covs->pitch.m21, covs->pitch.m22);
    float c11 = (-covs->pitch.m12*covs->pitch.m21 + covs->pitch.m11*(covs->pitch.m22+r_vals->pitch))/(covs->pitch.m22 + r_vals->pitch);
    float c12 = (covs->pitch.m12*r_vals->pitch)/(covs->pitch.m22 + r_vals->pitch);
    float c21 = (covs->pitch.m21*r_vals->pitch)/(covs->pitch.m22 + r_vals->pitch);
    float c22 = (covs->pitch.m22*r_vals->pitch)/(covs->pitch.m22 + r_vals->pitch);

    covs->pitch.m11 = c11;
    covs->pitch.m12 = c12;
    covs->pitch.m21 = c21;
    covs->pitch.m22 = c22;

    float c33 = covs->wheel.m11 + covs->wheel.m22 * pow(gain1_w,2) * pow(k,2) - covs->wheel.m12*gain1_w*k - covs->wheel.m21*gain1_w*k + pow(gain1_w,2)*r_vals->wheel;
    float c34 = covs->wheel.m12 + covs->wheel.m22 *gain1_w*gain2_w * pow(k,2) - covs->wheel.m22*gain1_w*k - covs->wheel.m12*gain2_w*k + gain1_w*gain2_w*r_vals->wheel;
    float c43 = covs->wheel.m21 + covs->wheel.m22 * gain1_w*gain2_w * pow(k,2) - covs->wheel.m22*gain1_w*k - covs->wheel.m21*gain2_w*k + gain1_w*gain2_w*r_vals->wheel;
    float c44 = covs->wheel.m22 + covs->wheel.m22*pow(gain2_w,2) * pow(k,2) - 2*covs->wheel.m22*gain2_w*k + pow(gain2_w,2)*r_vals->wheel;

    covs->wheel.m11 = c33;
    covs->wheel.m12 = c34;
    covs->wheel.m21 = c43;
    covs->wheel.m22 = c44;
    
    //printf("Update after covs: %0.15f %0.15f %0.15f %0.15f\n", covs->pitch.m11, covs->pitch.m12, covs->pitch.m21, covs->pitch.m22);

}


//Kalman filter for roll, essentially the same as for pitch

void roll_kalman_filter_update(float sensor, float dt, States *s, Matrix *q, Covariances *covs)
{   
    /* sensor is angular velocity */
    float diff = sensor - s->x2;

    /* Gain calculation */
    float gain1 = covs->roll.m12 * (1 / (covs->roll.m22 + r_vals->roll));
    float gain2 = covs->roll.m22 * (1 / (covs->roll.m22 + r_vals->roll));

    /* Update states */
    s->x1 = s->x1 + gain1 * diff;
    s->x2 = s->x2 + gain2 * diff;

    /* Covariance update */
    float c11 = (-covs->roll.m12*covs->roll.m21 + covs->roll.m11*(covs->roll.m22+r_vals->roll))/(covs->roll.m22 + r_vals->roll);
    float c12 = (covs->roll.m12*r_vals->roll)/(covs->roll.m22 + r_vals->roll);
    float c21 = (covs->roll.m21*r_vals->roll)/(covs->roll.m22 + r_vals->roll);
    float c22 = (covs->roll.m22*r_vals->roll)/(covs->roll.m22 + r_vals->roll);

    covs->roll.m11 = c11;
    covs->roll.m12 = c12;
    covs->roll.m21 = c21;
    covs->roll.m22 = c22;

}

void roll_kalman_filter_predict(float input, float dt, States *s, Matrix *q_t, Matrix *q_w, Covariances *covs)
{
    float X11 = s->x1 + dt * s->x2;
    float X21 = s->x2;

    s->x1 = X11;

    /* Dot product covariance and state transistion model*/
    float c11 = covs->roll.m11 + (dt * covs->roll.m21);
    float c12 = covs->roll.m12 + (dt * covs->roll.m22);
    float c21 = covs->roll.m21;
    float c22 = covs->roll.m22;

    /* c matrix transposed with state transistion model
    Fm11 Fm12    Fm11 Fm21
    Fm21 Fm22 => Fm12 Fm22
    */
    float p11 = c11 + c12 * dt;
    float p12 = c12;
    float p21 = c21 + c22;
    float p22 = c22;

    covs->roll.m11 = p11 + q_t->m11;  
    covs->roll.m12 = p12 + q_t->m12;
    covs->roll.m21 = p21 + q_t->m21;
    covs->roll.m22 = p22 + q_t->m22;
}