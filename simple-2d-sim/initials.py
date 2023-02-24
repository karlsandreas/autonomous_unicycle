from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman_A import KalmanFilterA 

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
    top_mass = 2.5,
    motor_reaction_speed = 0.1,
    sensor_position= 1.0,
    wheel_inertia= 0.114,
    top_inertia=0.125, 
)

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = LookaheadSpeedRegulator(
    params=DEFAULT_PARAMETERS,
    setpoint_x_d=-1.,
)

dt = 0.001

#State transition model
F = np.array([[1, dt],
            [0, 1]])
#Observation matrix
H = np.array([0,1]).reshape(1,2)  
#Process noise uncertainty, the uncertainty in how the unicycle is moving
Q = 0.5 * np.array([[(dt**4)/4, (dt**3)/2],
                            [(dt**3)/2, dt**2]])
#Measurement uncertainty, sensor uncertainty
R = np.array([[0.5]]).reshape(1,1) 
#Sensor distance from wheel center
R = DEFAULT_PARAMETERS.sensor_position
#Contoll matrix
G = np.array([(0.5*dt**2)*R,dt*R]).reshape(2,1)

#Inital states for kalman filter
x0 = np.array([INIT_STATE.top_angle,
               INIT_STATE.top_angle_d]).reshape(2,1)

DEFAULT_KALMAN = KalmanFilterA(
                    F = F,
                    G = G,
                    H = H,
                    Q = Q,
                    R = R)

DEFAULT_KALMAN_GAIN = 0.5