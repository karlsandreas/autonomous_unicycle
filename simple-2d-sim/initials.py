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
    top_height = 0.25,
    top_mass = 3.0,
    motor_reaction_speed = 0.9,
    sensor_position= 0.6,
    wheel_inertia= 0.114,
    top_inertia=0.125 + 5.0*0.5**2,
)

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = LookaheadSpeedRegulator(
    params=DEFAULT_PARAMETERS,
    setpoint_x_d=0.0,
)

DEFAULT_REG_PID = PIDController(
    kp = 23.0,
    ki = 0.0,
    kd = 2.0
)

if __name__ == "__main__":
    DEFAULT_REG.dump_params()

dt = 0.001

#state vector
#x = [theta, theta_d, x_d]


#State transition model
F = np.array([[1,dt,0,0],
              [0, 1,0,0],
              [0, 0,1,dt],
              [0, 0,0,1]])


F_t = np.array([[1, dt],
                [0, 1]])

F_w = np.array([[1, dt],
                [0, 1]])
#Observation matrix
#H = np.array([0, 1, DEFAULT_PARAMETERS.wheel_rad*np.pi/30]).reshape(1,3)
H = np.array([[0,1,0,0],
              [0,0,0,(1/DEFAULT_PARAMETERS.wheel_rad)*30/np.pi]])


H_t = np.array([0,1]).reshape(1,2)  
H_w = np.array([0,(1/DEFAULT_PARAMETERS.wheel_rad)*30/np.pi]).reshape(1,2)  





#Process noise uncertainty, the uncertainty in how the unicycle is moving
#High process noise seems to give better results, low process noise seems to give unstable and oscilating signal
q_w = 1000
q_t = 10

Q =  np.array([[q_t * (dt**4)/4, q_t * (dt**3)/2, 0, 0],
                     [q_t * (dt**3)/2,  q_t * dt**2, 0, 0],
                     [0, 0, q_w * (dt**4)/4, q_w * (dt**3)/2],
                     [0, 0, q_w * (dt**3)/2, q_w * dt**2]])
                     

Q_alt = np.array([[0.035,0,0,0],
                  [0,0.0007,0,0],
                  [0, 0, q_w * (dt**4)/4, q_w * (dt**3)/2],
                   [0, 0, q_w * (dt**3)/2, q_w * dt**2]])

Q_t = 10.0 * np.array([[(dt**4)/4, (dt**3)/2],
                     [(dt**3)/2,     dt**2]])

Q_w = 100.0 * np.array([[(dt**4)/4, (dt**3)/2],
                     [(dt**3)/2,     dt**2]])

#Measurement uncertainty, sensor uncertainty
R = np.array([[0.3,0],
              [0,20]])
R_t = 10 * np.array([[1]]).reshape(1,1) 
R_w = 1000000 * np.array([[1]]).reshape(1,1) 
R_s = DEFAULT_PARAMETERS.sensor_position
#Contoll matrix
#G = np.array([(0.5*dt**2)*R,dt*R])
#G = np.array([(0.5*dt**2)*R,dt*R]).reshape(2,1)

#Inital states for kalman filter
x0 = np.array([INIT_STATE.top_angle,
               INIT_STATE.top_angle_d]).reshape(2,1)

P = 1000 * np.eye(4,4)

P_t = np.array([[100, 0],
              [0, 100]])

P_w = np.array([[100,0],
                [0,100]])

E = np.array([[DEFAULT_PARAMETERS.I_w + (DEFAULT_PARAMETERS.wheel_mass + DEFAULT_PARAMETERS.top_mass)*DEFAULT_PARAMETERS.wheel_rad**2, DEFAULT_PARAMETERS.top_mass * DEFAULT_PARAMETERS.wheel_rad * DEFAULT_PARAMETERS.top_height**2],
             [DEFAULT_PARAMETERS.top_mass * DEFAULT_PARAMETERS.wheel_rad * DEFAULT_PARAMETERS.top_height**2, DEFAULT_PARAMETERS.I_c + DEFAULT_PARAMETERS.top_mass*DEFAULT_PARAMETERS.top_height**2]])
J = np.array([[1],
              [-1]])

K = np.dot(-np.linalg.inv(E),J)

#Control input
B = np.array([0,K[0][0],0,K[1][0]/DEFAULT_PARAMETERS.wheel_rad]).reshape(4,1)



DEFAULT_KALMAN_WHEEL = KalmanFilter(
                    F = F_w,
                    H = H_w,
                    Q = Q_w,
                    R = R_w
)

DEFAULT_KALMAN = KalmanFilter(
                    F = F,
                    H = H,
                    #G = -B,
                    Q = Q,
                    R = R,
                    P = P)

DEFAULT_KALMAN_GAIN = 0.5
