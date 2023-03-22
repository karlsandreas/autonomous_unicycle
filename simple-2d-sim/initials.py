from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from pidcontroller import PIDController

from kalman import KalmanFilter 


import numpy as np


INIT_STATE = SimulationState(
    wheel_position = 0,
    wheel_position_d = 0,
    top_angle = 0,
    top_angle_d = 0,
    motor_torque = 0,
)

DEFAULT_PARAMETERS = SimulationParameters(
    wheel_rad = 0.28,
    wheel_mass = 9.292,
    top_height = 0.4,
    top_mass = 2.5 + 5.0,
    motor_reaction_speed = 0.9,
    sensor_position= 0.4,
    wheel_inertia= 0.114,
    top_inertia=0.125 + 5.0*0.5**2, 
)

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = LookaheadSpeedRegulator(
    params=DEFAULT_PARAMETERS,
    setpoint_x_d=0.5,
)

DEFAULT_REG_PID = PIDController(
    kp = 23.0,
    ki = 0.0,
    kd = 2.0
)

dt = 0.001

#state vector
#x = [theta, theta_d, x_d]


#State transition model
F = np.array([[1, dt],
              [0, 1]])

F_w = np.array([[1]])
#Observation matrix
#H = np.array([0, 1, DEFAULT_PARAMETERS.wheel_rad*np.pi/30]).reshape(1,3)
H = np.array([0,1]).reshape(1,2)  
H_w = np.array([DEFAULT_PARAMETERS.wheel_rad*np.pi/30]).reshape(1,1)  





#Process noise uncertainty, the uncertainty in how the unicycle is moving
Q = 10.0 * np.array([[(dt**4)/4, (dt**3)/2],
                     [(dt**3)/2,     dt**2]])

Q_w = 10.0 * np.array([dt])
#Measurement uncertainty, sensor uncertainty
#Sensor distance from wheel center
R = np.array([[0.25]]).reshape(1,1) 
R_w = np.array([[0.25]]).reshape(1,1) 
R_s = DEFAULT_PARAMETERS.sensor_position
#Contoll matrix
#G = np.array([(0.5*dt**2)*R,dt*R])
#G = np.array([(0.5*dt**2)*R,dt*R]).reshape(2,1)

#Inital states for kalman filter
x0 = np.array([INIT_STATE.top_angle,
               INIT_STATE.top_angle_d]).reshape(2,1)

DEFAULT_KALMAN_WHEEL = KalmanFilter(
                    F = F_w,
                    H = H_w,
                    Q = Q_w,
                    R = R_w
)

DEFAULT_KALMAN = KalmanFilter(
                    F = F,
                    #G = G,
                    H = H,
                    Q = Q,
                    R = R)

DEFAULT_KALMAN_GAIN = 0.5