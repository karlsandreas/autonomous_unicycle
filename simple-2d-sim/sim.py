from __future__ import annotations

import numpy as np
import math

BOUNCE_FACTOR = 0.3

# Structure of simulation
# SimulationState contains all time-dependent variables of the system which can be simulated
# ControlSignals contain non-constant parameters such as signals produced by control systems externally
# SimulationParameters contains all constant parameters which define the simulation
#
# Simulator is a class which has the ability to simulate a SimulationState
# It contains an instance of SimulationParameters but the state to simulate has to be provided to the step method

class SimulationParameters:
    def __init__(
        self,
        wheel_rad: float, # [m], Radius of wheel
        wheel_mass: float, # [kg] Mass of wheel
        top_height: float, # [m], Distance from wheel center to top
        top_mass: float, # [kg] Mass of top
        motor_reaction_speed: float, # [s] Motor reaction speed

        g: float = 9.82, # Gravity
    ):
        self.wheel_rad = wheel_rad
        self.wheel_mass = wheel_mass
        self.top_height = top_height
        self.top_mass = top_mass
        self.motor_reaction_speed = motor_reaction_speed
        self.g = g


        m_w = wheel_mass
        r = wheel_rad
        m_c = top_mass
        l = top_height


        self.I_c = top_mass * top_height ** 2 #top inertia
        self.I_w = wheel_mass * wheel_rad**2 #Wheel inertia

        #X = [phi, phi_d, theta, theta_d]
        #u = torque 
        E = np.array([
            [self.I_w + (m_w + m_c)*r**2, m_c*r*l],
            [m_c*r*l, self.I_c + m_c*l**2]
            ])
        
        G = np.array([0, -m_c*g*l])
        F = np.array([])
        A_1 = np.dot(- np.linalg.inv(E), G)
        
    

        self.A = np.array([
            [0, 1, 0, 0],
            [A_1[0], 0, 0, 0],
            [0, 0, 0, 1],
            [0, A_1[1], 0,0]])

        H = np.array([1, -1])
        B_1 = np.dot(- np.linalg.inv(E), H.T)
        self.B = np.array([0, B_1[0], 0, B_1[1]]).T
        self.C = [[r, 0, 0, 0],
             [0, 0, 1, 0]] 




# Grouping all control signals
class ControlSignals:
    def __init__(
        self,
        motor_torque_signal: float = 0,
    ):
        self.motor_torque_signal = motor_torque_signal

# Convention: variables ending with _d represent the time derivative of the corresponding variable. _dd is second time derivative etc.
class SimulationState:
    def __init__(
        self,
        wheel_position: float, # [m] Position of the wheel
        wheel_position_d: float, # [ms^-1] Velocity of the wheel

        top_angle: float, # [rad] Angle of the top with repsect to the center of the wheel, zero being straight upwards
        top_angle_d: float, # [rad s^-1] Rotational velocity of the top with respect to the wheel

        motor_torque: float, # [Nm] Instantaneous of the motor
    ):

        self.wheel_position = wheel_position
        self.wheel_position_d = wheel_position_d
        self.top_angle = top_angle
        self.top_angle_d = top_angle_d
        self.motor_torque = motor_torque

    def apply_velocities(self, vels: SimulationState, dt: float) -> SimulationState:
        return SimulationState(
            wheel_position=self.wheel_position + vels.wheel_position * dt,
            wheel_position_d=self.wheel_position_d + vels.wheel_position_d * dt,
            top_angle=self.top_angle + vels.top_angle * dt,
            top_angle_d=self.top_angle_d + vels.top_angle_d * dt,
            motor_torque=self.motor_torque + vels.motor_torque * dt,
        )

# Uses Runge-Kutta-4 to integrate the simulation
class Simulator:
    def __init__(self, params: SimulationParameters):
        self.params = params

    # Returns a new SimulationState where each parameter has the value of it's derivative
    def state_derivative(self, state: SimulationState, signals: ControlSignals) -> SimulationState:
        I_c = self.params.I_c #top inertia
        I_w = self.params.I_w #Wheel inertia

        M = signals.motor_torque_signal / self.params.motor_reaction_speed
        m_c = self.params.top_mass
        r = self.params.wheel_rad
        l = self.params.top_height
        o1 = state.top_angle
        g = self.params.g
        do1 = state.top_angle_d
        m_w = self.params.wheel_mass

        
        wheel_angle_dd = (-M - (m_c*r*l*np.cos(o1) * (M + l*m_c*g*np.sin(o1))/(m_c*(l**2)+I_c)) + m_c*r*l*np.sin(o1)*(do1**2)) / \
               (m_c*r**2 - ((r*l*m_c*np.cos(o1))**2)/(m_c*l**2 + I_c) + m_w*r**2 + I_w)

        top_angle_dd = (M - m_c*r*wheel_angle_dd*l*np.cos(o1) + m_c*g*l*np.sin(o1))/(m_c*l**2 + I_c)
        
        wheel_position_dd = r*wheel_angle_dd

        # All entries are derivatives
        return SimulationState(
            wheel_position = state.wheel_position_d,
            wheel_position_d = wheel_position_dd,
            top_angle = state.top_angle_d,
            top_angle_d = top_angle_dd,
            motor_torque= (signals.motor_torque_signal - state.motor_torque) / self.params.motor_reaction_speed
        )
            

    # Enforces limit conditions (i.e. top bouncing on floor)
    # Modifies state in-place
    def limit(self, state: SimulationState, signals: ControlSignals):
        max_angle = math.pi - math.acos(self.params.wheel_rad / self.params.top_height)

        if state.top_angle > max_angle:
            state.top_angle = max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.wheel_position_d *= BOUNCE_FACTOR

        if state.top_angle < -max_angle:
            state.top_angle = -max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.wheel_position_d *= BOUNCE_FACTOR

    # Runs one step of Runge-Kutta 4 (classic)
    # Does not modify in-place, instead returns next state
    def step(self, state: SimulationState, signals: ControlSignals, dt: float) -> SimulationState:
        k1 = self.state_derivative(state, signals)
        k2 = self.state_derivative(state.apply_velocities(k1, dt / 2), signals)
        k3 = self.state_derivative(state.apply_velocities(k2, dt / 2), signals)
        k4 = self.state_derivative(state.apply_velocities(k3, dt), signals)
        res = state.apply_velocities(k1, dt/6).apply_velocities(k2, dt/3).apply_velocities(k3, dt/3).apply_velocities(k4, dt/6)
        self.limit(res, signals)
        return res
