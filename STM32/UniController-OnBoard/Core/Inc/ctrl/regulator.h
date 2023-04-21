
#ifndef INC_CTRL_REGULATOR_H_
#define INC_CTRL_REGULATOR_H_

// #define param_A -2.326789980879802F //-1.6492579658751876F
// #define param_B -0.3210967164801556F //-0.48285301954085175F
// #define param_C 14.151599977724041F //21.84620564928607F
// #define param_D 0.7624427764817346F //2.824481941982574F
// #define param_E 3.6330438924976995F //2.0854114637561922F

#define param_A -0.6135288166011171
#define param_B -0.29096653216753593
#define param_C 7.728539864148976
#define param_D 0.8081233571611377
#define param_E 2.16914838615865


float LookaheadSpeedRegulator(float setpoint_x_d, float theta, float theta_d, float x_d, float dt);

#endif
