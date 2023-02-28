#include <math.h>

void pitch_kalman_filter_predict(float input, float dt, States *s, Matrix *cov, Matrix *q)
{
    /* input is acceleration input*/

    /* Control matrix times control input, acceleration*/
    float U1 = 0.5*pow(dt,2) * input * SENSOR_POS_Z;
    float U2 = dt * input * SENSOR_POS_Z;
    
    float F12 = dt;

    float X11 = F11 * s->x1 + F12 * s->x2;
    float X21 = F22 * s->x2;

    s->x1 = X11 + U1;
    s->x2 = X21 + U2;
    /*
    c matrix = cm11 cm12
               cm21 cm22
    */

    /* Dot product covariance and state transistion model*/
    float c11 = (F11 * cov->m11 + F12 * cov->m21);
    float c12 = (F11 * cov->m12 + F12 * cov->m22);
    float c21 = (F22 * cov->m21);
    float c22 = (F22 * cov->m22);
    /* c matrix transposed with state transistion model
    Fm11 Fm12    Fm11 Fm21
    Fm21 Fm22 => Fm12 Fm22
    */
    float p11 = c11 * F11 + c12 * F12;
    float p12 = c11 * F21 + c12 * F22;
    float p21 = c21 * F11 + c22 * F12;
    float p22 = c21 * F21 + c22 * F22;
 
    cov->m11 = p11 + q->m11;  
    cov->m12 = p12 + q->m12;
    cov->m21 = p21 + q->m21;
    cov->m22 = p22 + q->m22;
}

void kalman_filter_update(float sensor, float dt, States *s, Matrix *cov, Matrix *q)
{
    /* sensor is angular velocity */
    float diff = sensor - s->x2;

    /* Gain calculation */
    float gain = cov->m22 * (1 / (cov->m22 + KALMAN_R));

    /* Update states */
    s->x1 = s->x1 + gain * diff;

    /* Covariance update */
    float c11 = cov->m22*cov->m11/(cov->m22+KALMAN_R) * (cov->m11 / (cov->m22 + KALMAN_R)) + pow((cov->m11 / (cov->m22 + KALMAN_R)),2) * KALMAN_R;
    float c12 = cov->m22*cov->m11/(cov->m22+KALMAN_R) * (cov->m22 / (cov->m22 + KALMAN_R)) + (cov->m11 / (cov->m22 + KALMAN_R)) * KALMAN_R * (cov->m22 / (cov->m22 + KALMAN_R));
    float c21 = pow(cov->m22,2)/(cov->m22+KALMAN_R) * (cov->m11 / (cov->m22 + KALMAN_R)) + (cov->m11 / (cov->m22 + KALMAN_R)) * KALMAN_R * (cov->m22 / (cov->m22 + KALMAN_R));
    float c22 = pow(cov->m22,2)/(cov->m22+KALMAN_R) * (cov->m22 / (cov->m22 + KALMAN_R)) + pow((cov->m22 / (cov->m22 + KALMAN_R)),2) * KALMAN_R;

    cov->m11 = c11;
    cov->m12 = c12;
    cov->m21 = c21;
    cov->m22 = c22;

}