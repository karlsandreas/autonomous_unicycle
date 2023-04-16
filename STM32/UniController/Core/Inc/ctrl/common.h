
#ifndef INC_CTRL_COMMON_H_
#define INC_CTRL_COMMON_H_

//#include <math.h>

struct states
{
    float x1; /* theta angle pitch*/
    float x2; /* theta_d angular velocity pitch*/
    float x3; /* x distance */
    float x4; /* x_d velocity */
    float x5; /* theta angle roll*/
    float x6; /* theta_d angular velocity roll*/
};

typedef struct states States;

struct matrix
{   
    float m11;
    float m12;
    float m21;
    float m22;
};

typedef struct matrix Matrix;

struct covariances
{   
    Matrix pitch;
    Matrix roll;
    Matrix wheel
     
};

typedef struct covariances Covariances;

struct r_error  
{
    float pitch;
    float roll;
    float wheel;
};

typedef struct r_error R_error;

#define SENSOR_POS_Z (0.4)
#define F11 (1.0)
/* F1_2 = dt*/
#define F21 (0.0)
#define F22 (1.0)

#define R_ANGLE (0.01F)
#define R_WHEEL (0.001F)

#define WHEEL_RAD (0.28)
#define RPM_TO_RADS (30/M_PI)  

#endif
