from sim import SimulationState_Roll, SimulationState_Pitch, SimulationParameters, ControlSignals, Simulator_Pitch, Simulator_Roll
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from pidcontroller import PIDController

from kalman import KalmanFilter 


import numpy as np


INIT_STATE_P = SimulationState_Pitch(
    wheel_position = 0,
    wheel_position_d = 0,
    top_angle = 0,
    top_angle_d = 0,
    motor_torque = 0,
)

INIT_STATE_R = SimulationState_Roll(
    top_angle= 0.0,
    top_angle_d= 0.0,
    reaction_wheel_angle= 0.0,
    reaction_wheel_angle_d= 0.0,
    motor_torque= 0.0,   
)

DEFAULT_PARAMETERS = SimulationParameters(
    chassi_mass= 3.0,
    pitch_wheel_mass= 9.292,
    roll_wheel_mass= 4.0,
    yaw_wheel_mass= 0.0,
    
    pitch_wheel_rad= 0.28,
    roll_wheel_rad= 0.2,
    yaw_wheel_rad= 0.0,

    chassi_mc_height= 0.25,
    roll_wheel_height= 1.0,
    yaw_wheel_height= 0.0,
    
    motor_reaction_speed = 0.95,
    sensor_position= 0.6,

    total_intertia_roll=1+ 0.125 + 5.0*0.5**2,
    pitch_wheel_inertia= 0.114,
    roll_wheel_inertia= 0.020,
    yaw_wheel_intertia= 0.0,  
    chassi_inertia=0.125 + 5.0*0.5**2,
)

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = LookaheadSpeedRegulator(
    params=DEFAULT_PARAMETERS,
    setpoint_x_d=0.0,
)

DEFAULT_REG_PID = PIDController(
    kp = 0.4,
    ki = 0.0,
    kd = 0.0
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
              [0,0,0,(1/DEFAULT_PARAMETERS.pitch_wheel_rad)*30/np.pi]])


H_t = np.array([0,1]).reshape(1,2)  
H_w = np.array([0,(1/DEFAULT_PARAMETERS.pitch_wheel_rad)*30/np.pi]).reshape(1,2)  





#Process noise uncertainty, the uncertainty in how the unicycle is moving
#High process noise seems to give better results, low process noise seems to give unstable and oscilating signal
q_wc = 100
q_tc = 80

Q =  np.array([[q_tc * (dt**4)/4, q_tc * (dt**3)/2, 0, 0],
                     [q_tc * (dt**3)/2,  q_tc * dt**2, 0, 0],
                     [0, 0, q_wc * (dt**4)/4, q_wc * (dt**3)/2],
                     [0, 0, q_wc * (dt**3)/2, q_wc * dt**2]])
                     

Q_alt = np.array([[0.035,0,0,0],
                  [0,0.0007,0,0],
                  [0, 0, q_wc * (dt**4)/4, q_wc * (dt**3)/2],
                   [0, 0, q_wc * (dt**3)/2, q_wc * dt**2]])

#Q for pitch top
Q_t = q_tc * np.array([[(dt**4)/4, (dt**3)/2],
                     [(dt**3)/2,     dt**2]])


#Q for roll
Q_r = q_tc * np.array([[(dt**4)/4, (dt**3)/2],
                     [(dt**3)/2,     dt**2]])

#Q for wheel
Q_w = q_wc * np.array([[(dt**4)/4, (dt**3)/2],
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
x0 = np.array([INIT_STATE_P.top_angle,
               INIT_STATE_P.top_angle_d]).reshape(2,1)

P = 1000 * np.eye(4,4)

P_t = np.array([[100, 0],
              [0, 100]])

P_w = np.array([[100,0],
                [0,100]])

E = np.array([[DEFAULT_PARAMETERS.I_hp + (DEFAULT_PARAMETERS.pitch_wheel_mass + DEFAULT_PARAMETERS.chassi_mass)*DEFAULT_PARAMETERS.pitch_wheel_rad**2, DEFAULT_PARAMETERS.chassi_mass * DEFAULT_PARAMETERS.pitch_wheel_rad * DEFAULT_PARAMETERS.chassi_mc_height**2],
             [DEFAULT_PARAMETERS.chassi_mass * DEFAULT_PARAMETERS.pitch_wheel_rad * DEFAULT_PARAMETERS.chassi_mc_height**2, DEFAULT_PARAMETERS.I_cp + DEFAULT_PARAMETERS.chassi_mass*DEFAULT_PARAMETERS.chassi_mc_height**2]])
J = np.array([[1],
              [-1]])

K = np.dot(-np.linalg.inv(E),J)

#Control input
B = np.array([0,K[0][0],0,K[1][0]/DEFAULT_PARAMETERS.pitch_wheel_rad]).reshape(4,1)



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
                    R = R)

DEFAULT_KALMAN_GAIN = 0.5
