from __future__ import annotations
from typing import Tuple
from abc import ABC, abstractmethod

import random

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
        chassi_mass: float, #[kg] mass for chassi 
        pitch_wheel_mass: float, # [kg] Mass of pitch wheel, drive wheel
        roll_wheel_mass: float, # [kg], mass of roll wheel
        yaw_wheel_mass: float, # [kg], mass of yaw wheel, not used atm

        pitch_wheel_rad: float, # [m], Radius of pitch wheel, drive wheel
        roll_wheel_rad: float, # [m], radius of roll wheel
        yaw_wheel_rad: float, # [m], radius of yaw wheel, not used atm
        
        chassi_mc_height: float, # [m] distance from mass centre to "floor"
        roll_wheel_height: float, # [m] distance from roll wheel center to the floor
        yaw_wheel_height: float, # [m] distance from yaw wheel centre to the "floor"
                
        motor_reaction_speed: float, # [s] Motor reaction speed
        sensor_position: float, # [m] distance from wheel on top-axis

        total_intertia_roll: float,
        pitch_wheel_inertia: float,
        roll_wheel_inertia: float,
        yaw_wheel_intertia: float,
        chassi_inertia: float,
        
        g: float = 9.82, # Gravity
    ):
        self.pitch_wheel_rad = pitch_wheel_rad
        self.pitch_wheel_mass = pitch_wheel_mass
                
        #self.chassi_mc_height = chassi_mc_height
        #self.chassi_mass = chassi_mass
        self.motor_reaction_speed = motor_reaction_speed
        self.sensor_position = sensor_position
        self.chassi_mc_height = chassi_mc_height # [m] distance from mass centre to "floor"
        self.chassi_mass = chassi_mass
        self.g = g

        self.roll_wheel_rad = roll_wheel_rad
        self.roll_wheel_mass = roll_wheel_mass
        self.roll_wheel_height = roll_wheel_height # [m] distance from roll wheel center to the floor

        self.yaw_wheel_mass = yaw_wheel_mass
        self.yaw_wheel_rad = yaw_wheel_rad
        self.yaw_wheel_height = yaw_wheel_height # [m] distance from yaw wheel centre to the "floor"

        m_w = pitch_wheel_mass
        r = pitch_wheel_rad
        m_c = chassi_mass
        l = chassi_mc_height

        self.I_tr = total_intertia_roll # Inertia total for unicycle with regards to roll
        self.I_cp = chassi_inertia # Inertia chassi with regards to pitch axis  
        self.I_hp = pitch_wheel_inertia # Inertia reaction wheel pitch
        self.I_hr = roll_wheel_inertia # Inertia reaction wheel roll 
        self.I_hy = yaw_wheel_intertia # Inertia reaction wheel yaw 



    def abcd(self) -> Tuple[float, float, float, float]:
        m_w = self.pitch_wheel_mass
        r = self.pitch_wheel_rad
        m_c = self.chassi_mass
        l = self.chassi_mc_height

        X = self.I_hp + (m_w + m_c) * r**2
        Y = U = m_c*r*l
        V = self.I_cp + m_c * l**2
        Å = m_c * self.g * l
        d = X*V - U*Y

        # A φ'' + Y θ'' = Z
        # U φ'' + V θ'' = W

        A = r/d * (-Å*Y)
        B = r/d * (-V-Y)
        C = 1/d * (Å*X)
        D = 1/d * (U+X)
        return A, B, C, D
    

# Grouping all control signals
class ControlSignals:
    def __init__(
        self,
        motor_torque_signal_pitch: float = 0,
        motor_torque_signal_roll: float = 0,
    ):
        self.motor_torque_signal_pitch = motor_torque_signal_pitch
        self.motor_torque_signal_roll = motor_torque_signal_roll

class SimulationState(ABC):
    def __init__(self):
        pass

        



# Convention: variables ending with _d represent the time derivative of the corresponding variable. _dd is second time derivative etc.
class SimulationState_Pitch(SimulationState):
    def __init__(
        self,
        wheel_position: float = 0., # [m] Position of the wheel
        wheel_position_d: float = 0., # [ms^-1] Velocity of the wheel

        top_angle: float = 0., # [rad] Angle of the top with repsect to the center of the wheel, zero being straight upwards
        top_angle_d: float = 0., # [rad s^-1] Rotational velocity of the top with respect to the wheel

        motor_torque: float = 0., # [Nm] Instantaneous of the motor
    ):

        self.wheel_position = wheel_position
        self.wheel_position_d = wheel_position_d
        self.top_angle = top_angle
        self.top_angle_d = top_angle_d
        self.motor_torque = motor_torque

    def apply_velocities(self, vels: SimulationState_Pitch, dt: float) -> SimulationState_Pitch:
        return SimulationState_Pitch(
            wheel_position=self.wheel_position + vels.wheel_position * dt,
            wheel_position_d=self.wheel_position_d + vels.wheel_position_d * dt,
            top_angle=self.top_angle + vels.top_angle * dt,
            top_angle_d=self.top_angle_d + vels.top_angle_d * dt,
            motor_torque=self.motor_torque + vels.motor_torque * dt,
        )
    
    def get_array(self):
        return np.array([self.top_angle, self.top_angle_d, self.wheel_position, self.wheel_position_d, self.motor_torque])


# Uses Runge-Kutta-4 to integrate the simulation
class Simulator_Pitch:
    def __init__(self, params: SimulationParameters):
        self.params = params

    # Returns a new SimulationState where each parameter has the value of it's derivative
    def state_derivative(self, state: SimulationState_Pitch, signals: ControlSignals) -> SimulationState_Pitch:
        chassi_inertia = self.params.I_cp #top inertia
        pitch_wheel_inertia = self.params.I_hp #Wheel inertia

        M = signals.motor_torque_signal_pitch
        m_c = self.params.chassi_mass
        r = self.params.pitch_wheel_rad
        l = self.params.chassi_mc_height
        o1 = state.top_angle
        g = self.params.g
        do1 = state.top_angle_d
        m_w = self.params.pitch_wheel_mass
        phi_dot = state.wheel_position_d / r

        
        # wheel_angle_dd = (-M - (m_c*r*l*np.cos(o1) * (M + l*m_c*g*np.sin(o1))/(m_c*(l**2)+chassi_inertia)) + m_c*r*l*np.sin(o1)*(do1**2)) / \
        #        (m_c*r**2 - ((r*l*m_c*np.cos(o1))**2)/(m_c*l**2 + chassi_inertia) + m_w*r**2 + pitch_wheel_inertia)

        # top_angle_dd = (M - m_c*r*wheel_angle_dd*l*np.cos(o1) + m_c*g*l*np.sin(o1))/(m_c*l**2 + chassi_inertia)

        # We have two equations
        # X φ'' + Y θ'' = Z
        # U φ'' + V θ'' = W
        X, Y = pitch_wheel_inertia + (m_w+m_c)*r**2, m_c*r*l*np.cos(o1)
        Z = -M + m_c*r*l*np.sin(o1)*do1*phi_dot # Ska det inte vara do1^2 ?
        U, V = m_c*r*l*np.cos(o1), chassi_inertia + m_c*l**2
        W = M + m_c*g*l*np.sin(o1)

        # Determinant D
        D = X * V - U * Y

        # Solve
        phi_dd   = 1/D * ( V * Z - Y * W)
        theta_dd = 1/D * (-U * Z + X * W)

        wheel_position_dd = r*phi_dd

        # All entries are derivatives
        return SimulationState_Pitch(
            wheel_position = state.wheel_position_d,
            wheel_position_d = wheel_position_dd,
            top_angle = state.top_angle_d,
            top_angle_d = theta_dd,
            motor_torque= (signals.motor_torque_signal_pitch - state.motor_torque) / self.params.motor_reaction_speed
        )


    # Enforces limit conditions (i.e. top bouncing on floor)
    # Modifies state in-place
    def limit(self, state: SimulationState_Pitch, signals: ControlSignals):
        max_angle = math.pi - math.acos(self.params.chassi_mc_height / self.params.pitch_wheel_rad)

        if state.top_angle > max_angle:
            state.top_angle = max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.wheel_position_d *= BOUNCE_FACTOR

        if state.top_angle < -max_angle:
            state.top_angle = -max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.wheel_position_d *= BOUNCE_FACTOR

    # z = "up", x = "right" when upright
    def sensor_reading(self, state: SimulationState_Pitch, signals: ControlSignals, variances: Tuple(float,float,float,float)) -> np.array([float, float, float, float]):
        deriv = self.state_derivative(state, signals)
        wheel_position_dd = deriv.wheel_position_d
        top_angle_dd = deriv.top_angle_d

        a = self.params.sensor_position * top_angle_dd

        a_x = wheel_position_dd + self.params.sensor_position * top_angle_dd
        #a_x = wheel_position_dd + a * np.cos(state.top_angle)
        #a_z = a * np.sin(state.top_angle)
        a_z = wheel_position_dd * math.sin(state.top_angle) - self.params.sensor_position * state.top_angle_d ** 2
        
        
        top_angle_d = state.top_angle_d + random.gauss(0, variances[0])
        wheel_rpm = ms_to_rpm( state.wheel_position_d, self.params.pitch_wheel_rad) + random.gauss(0,variances[1]) 
        return np.array([top_angle_d, wheel_rpm, a_x , a_z])

    

    # Returns (a_hat_x, a_hat_z), basis vectors for the sensor's frame of reference
    def sensor_axes(self, state: SimulationState_Pitch) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        a_hat_x = (math.cos(state.top_angle), -math.sin(state.top_angle))
        a_hat_z = (math.sin(state.top_angle), math.cos(state.top_angle))
        return a_hat_x, a_hat_z

    # Runs one step of Runge-Kutta 4 (classic)
    # Does not modify in-place, instead returns next state
    def step(self, state: SimulationState_Pitch, signals: ControlSignals, dt: float) -> SimulationState_Pitch:
        k1 = self.state_derivative(state, signals)
        k2 = self.state_derivative(state.apply_velocities(k1, dt / 2), signals)
        k3 = self.state_derivative(state.apply_velocities(k2, dt / 2), signals)
        k4 = self.state_derivative(state.apply_velocities(k3, dt), signals)
        res = state.apply_velocities(k1, dt/6).apply_velocities(k2, dt/3).apply_velocities(k3, dt/3).apply_velocities(k4, dt/6)
        self.limit(res, signals)
        return res

##Roll simulation related
    
class SimulationState_Roll(SimulationState):
    def __init__(
        self,
        top_angle: float, # [rad] Angle of the top with repsect to the center of the wheel, zero being straight upwards
        top_angle_d: float, # [rad s^-1] Rotational velocity of the top with respect to the wheel
        #gravity_torque: float,
        reaction_wheel_angle: float,
        reaction_wheel_angle_d: float,
        motor_torque: float,    #Reaktion wheel torque
    ):
        self.top_angle = top_angle
        self.top_angle_d = top_angle_d
        self.reaction_wheel_angle = reaction_wheel_angle
        self.reaction_wheel_angle_d = reaction_wheel_angle_d
        #self.gravity_torque = gravity_torque
        self.motor_torque = motor_torque

    def apply_velocities(self, vels: SimulationState_Roll, dt: float) -> SimulationState_Roll:
        return SimulationState_Roll(
            top_angle=self.top_angle + vels.top_angle * dt,
            top_angle_d=self.top_angle_d + vels.top_angle_d * dt,
            reaction_wheel_angle=self.reaction_wheel_angle + vels.reaction_wheel_angle * dt, 
            reaction_wheel_angle_d=self.reaction_wheel_angle_d + vels.reaction_wheel_angle_d * dt, 
            motor_torque=self.motor_torque + vels.motor_torque * dt,
        )

# Uses Runge-Kutta-4 to integrate the simulation
class Simulator_Roll():
    def __init__(self, params: SimulationParameters):
        self.params = params

    # Returns a new SimulationState where each parameter has the value of it's derivative
    def state_derivative(self, state_roll: SimulationState_Roll, state_pitch: SimulationState_Pitch,  signals: ControlSignals) -> SimulationState_Roll:

        motor_torque= (signals.motor_torque_signal_roll - state_roll.motor_torque) / self.params.motor_reaction_speed
        
        tau_r = motor_torque

        tau_gr0 = (self.params.pitch_wheel_mass * self.params.pitch_wheel_rad + self.params.chassi_mass * self.params.chassi_mc_height + \
                    self.params.yaw_wheel_mass * self.params.yaw_wheel_height + self.params.roll_wheel_mass * self.params.roll_wheel_height) * self.params.g

        tau_gr = tau_gr0 * math.sin(state_roll.top_angle)*math.cos(state_pitch.top_angle)
        top_angle_dd = (tau_gr - tau_r) / self.params.I_tr

        

        reaction_wheel_dd = tau_r / self.params.I_hr
        
        #M1 = self.params.pitch_wheel_mass * self.params.g * np.sin(state_roll.top_angle) * self.params.height
        
        #M2 = self.params.chassi_mass * self.params.g * np.sin(state_roll.top_angle) * self.params.height/2

        #top_angle_dd = (M1 + M2 - motor_torque) / self.params.I_a

        # All entries are derivatives
        
        return SimulationState_Roll(
            top_angle = state_roll.top_angle_d,
            top_angle_d = top_angle_dd,
            reaction_wheel_angle= state_roll.reaction_wheel_angle_d,
            reaction_wheel_angle_d= reaction_wheel_dd,
            motor_torque= motor_torque
        )
            

    # Enforces limit conditions (i.e. top bouncing on floor)
    # Modifies state in-place
    def limit(self, state: SimulationState_Roll, signals: ControlSignals):
        max_angle = math.pi - math.acos(-self.params.pitch_wheel_rad / self.params.roll_wheel_height)

        if state.top_angle > max_angle:
            state.top_angle = max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.motor_torque = 0

        if state.top_angle < -max_angle:
            state.top_angle = -max_angle
            state.top_angle_d *= -BOUNCE_FACTOR
            state.motor_torque = 0

    # Runs one step of Runge-Kutta 4 (classic)
    # Does not modify in-place, instead returns next state
    def step(self, state: SimulationState_Roll, state_p: SimulationState_Pitch, signals: ControlSignals, dt: float) -> SimulationState_Roll:
        k1 = self.state_derivative(state, state_p, signals)
        k2 = self.state_derivative(state.apply_velocities(k1, dt / 2), state_p, signals)
        k3 = self.state_derivative(state.apply_velocities(k2, dt / 2), state_p, signals)
        k4 = self.state_derivative(state.apply_velocities(k3, dt), state_p, signals)
        res = state.apply_velocities(k1, dt/6).apply_velocities(k2, dt/3).apply_velocities(k3, dt/3).apply_velocities(k4, dt/6)
        self.limit(res, signals)
        return res
    
    def get_pos_vel(self, state: SimulationState_Roll):
        
        pos_x = self.params.roll_wheel_height * np.sin(state.top_angle) 
        pos_z = self.params.roll_wheel_height * np.cos(state.top_angle)
        pos_x_d = self.params.roll_wheel_height * state.top_angle_d * np.cos(state.top_angle)
        pos_z_d = -self.params.roll_wheel_height * state.top_angle_d * np.sin(state.top_angle)

        return PositionState(
                            top_center_pos_x = pos_x,
                            top_center_pos_z = pos_z,
                            top_center_pos_x_d = pos_x_d,
                            top_center_pos_z_d = pos_z_d)

class PositionState:
    def __init__(self,
                top_center_pos_x: float,
                top_center_pos_z: float,
                top_center_pos_x_d: float,
                top_center_pos_z_d: float,
    ):
        self.top_center_pos_x = top_center_pos_x
        self.top_center_pos_z = top_center_pos_z
        self.top_center_pos_x_d = top_center_pos_x_d
        self.top_center_pos_z_d = top_center_pos_z_d
    

def ms_to_rpm(velocity: float, radius: float):
        return (30/(math.pi*radius) * velocity) 

def rpm_to_ms(rpm: float, radius: float):
        return ((math.pi*radius* rpm)/30)