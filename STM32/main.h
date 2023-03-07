struct states
{
    float x1; /* theta angle*/
    float x2; /* theta_d angular velocity*/
    float x3; /* x distance */
    float x4; /* x_d velocity */
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

#define SENSOR_POS_Z (0.4)
#define F11 (1.0)
/* F1_2 = dt*/
#define F21 (0.0)
#define F22 (1.0)

#define E_PITCH (0.01F)
#define E_WHEEL (0.001F)

