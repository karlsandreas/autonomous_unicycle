#include <math.h>
#include "main.h"
//#include <stdio.h>


float cov11 = 1.0;
float cov12 = 0.0;
float cov21 = 1.0;
float cov22 = 0.0;


void pitch_kalman_filter_predict(float input, float dt, States *s, Matrix *q)
{
    /* input is acceleration input*/

    /* Control matrix times control input, acceleration*/
    float U1 = 0.5*pow(dt,2) * input * SENSOR_POS_Z;
    float U2 = dt * input * SENSOR_POS_Z;
    //printf("Input: %f\n", input);
    float F12 = dt;

    float X11 = F11 * s->x1 + F12 * s->x2;
    float X21 = F22 * s->x2;
    //printf("Us: %0.10f %0.10f\n", U1, U2);
    s->x1 = X11 + U1;
    s->x2 = X21 + U2;
    /*
    c matrix = cov11 cov12
               cov21 cov22
    */

    /* Dot product covariance and state transistion model*/
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11, cov12, cov21, cov22);
    float c11 = (F11 * cov11) + (F12 * cov21);
    float c12 = (F11 * cov12) + (F12 * cov22);
    float c21 = (F22 * cov21);
    float c22 = (F22 * cov22);
    //printf("Cs: %0.15f %0.15f %0.15f %0.15f\n", c11, c12, c21, c22);
    /* c matrix transposed with state transistion model
    Fm11 Fm12    Fm11 Fm21
    Fm21 Fm22 => Fm12 Fm22
    */
    float p11 = (c11 * F11) + (c12 * F12);
    float p12 = (c11 * F21) + (c12 * F22);
    float p21 = c21 * F11 + c22 * F12;
    float p22 = c21 * F21 + c22 * F22;
    //printf("Ps: %0.15f %0.15f %0.15f %0.15f \n", p11, p12, p21, p22);
 
    cov11 = p11 + q->m11;  
    cov12 = p12 + q->m12;
    cov21 = p21 + q->m21;
    cov22 = p22 + q->m22;
    //printf("Covs: %0.15f %0.15f %0.15f %0.15f\n", cov11, cov12, cov21, cov22);
}

void kalman_filter_update(float sensor, float dt, States *s, Matrix *q)
{
    /* sensor is angular velocity */
    float diff = sensor - s->x2;
    //printf("Diff: %f\n", diff);

    /* Gain calculation */
    float gain1 = cov12 * (1 / (cov22 + KALMAN_R));
    float gain2 = cov22 * (1 / (cov22 + KALMAN_R));
    //printf("Gain1: %f Gain2: %f\n", gain1, gain2);
    /* Update states */
    s->x1 = s->x1 + gain1 * diff;
    s->x2 = s->x2 + gain2 * diff;
    //printf("New states: x1 = %f x2 = %f\n", s->x1, s->x2);

    /* Covariance update */
    //printf("Update covs: %0.15f %0.15f %0.15f %0.15f\n", cov11, cov12, cov21, cov22);
    float c11 = (-cov12*cov21 + cov11*(cov22+KALMAN_R))/(cov22 + KALMAN_R);
    float c12 = (cov12*KALMAN_R)/(cov22 + KALMAN_R);
    float c21 = (cov21*KALMAN_R)/(cov22 + KALMAN_R);
    float c22 = (cov22*KALMAN_R)/(cov22 + KALMAN_R);

    cov11 = c11;
    cov12 = c12;
    cov21 = c21;
    cov22 = c22;
    //printf("Update after covs: %0.15f %0.15f %0.15f %0.15f\n", cov11, cov12, cov21, cov22);

}