
#define param_A -2.326789980879802F //-1.6492579658751876F
#define param_B -0.3210967164801556F //-0.48285301954085175F
#define param_C 14.151599977724041F //21.84620564928607F
#define param_D 0.7624427764817346F //2.824481941982574F
#define param_E 3.6330438924976995F //2.0854114637561922F

float LookaheadSpeedRegulator(float setpoint_x_d, float theta, float theta_d, float x_d, float dt);

// Nested φ'-P θ-PD
typedef struct {
	float setpoint_theta_0;
	float kp1, kd1; // PD for roll angle
	float kp2; // P for wheel
} RollRegulator;

float roll_reg_setpoint_theta(RollRegulator *roll_reg, float dt, float theta, float theta_d, float wheel_rpm);
float roll_reg_step(RollRegulator *roll_reg, float dt, float theta, float theta_d, float wheel_rpm);

// Nested, x'-P θ-PD
typedef struct {
	float kp1, kd1; // PD for pitch angle θ
	float kp2; // P for x'
} PitchRegulator;

float pitch_reg_step(PitchRegulator *pitch_reg, float dt, float theta, float theta_d, float ground_speed);
