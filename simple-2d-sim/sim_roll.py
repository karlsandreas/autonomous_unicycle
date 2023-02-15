from __future__ import annotations
from typing import Tuple

import numpy as np
import math

BOUNCE_FACTOR = 0.1

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
        height: float, # [m], Distance from ground to top
        chassi_mass: float, # [kg] Mass of top
        motor_reaction_speed: float, # [s] Motor reaction speed

        g: float = 9.82, # Gravity
    ):
        self.wheel_rad = wheel_rad
        self.wheel_mass = wheel_mass
        self.height = height
        self.chassi_mass = chassi_mass
        self.motor_reaction_speed = motor_reaction_speed
        self.g = g 

        self.I_c = 1/12 * chassi_mass * (0.1**2 * height**2) #chassi intertia assuming solid block
        self.I_w = wheel_mass * wheel_rad**2 #Wheel inertia
        self.I_wa = wheel_mass * height**2 #Wheel inertia with respect to the point in contact with ground
        self.I_a = self.I_c + self.I_w + self.I_wa

    def abcd(self) -> Tuple[float, float, float, float]:
        A = 1.0
        B = 1.0
        C = 1.0
        D = 1.0
        return A, B, C, D


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
        top_angle: float, # [rad] Angle of the top with repsect to the center of the wheel, zero being straight upwards
        top_angle_d: float, # [rad s^-1] Rotational velocity of the top with respect to the wheel
        motor_torque: float,
    ):
        self.top_angle = top_angle
        self.top_angle_d = top_angle_d
        self.motor_torque = motor_torque

    def apply_velocities(self, vels: SimulationState, dt: float) -> SimulationState:
        return SimulationState(
            top_angle=self.top_angle + vels.top_angle * dt,
            top_angle_d=self.top_angle_d + vels.top_angle_d * dt,
            motor_torque=self.motor_torque + vels.motor_torque * dt,
        )
    
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
    


# Uses Runge-Kutta-4 to integrate the simulation
class Simulator:
    def __init__(self, params: SimulationParameters):
        self.params = params

    # Returns a new SimulationState where each parameter has the value of it's derivative
    def state_derivative(self, state: SimulationState, signal) -> SimulationState:
        motor_torque = signal #(signal - state.motor_torque) / self.params.motor_reaction_speed
        M1 = self.params.wheel_mass * self.params.g * np.sin(state.top_angle) * self.params.height
        M2 = self.params.chassi_mass * self.params.g * np.sin(state.top_angle) * self.params.height/2

        top_angle_dd = (M1 + M2 - motor_torque) / self.params.I_a

        # All entries are derivatives
        return SimulationState(
            top_angle = state.top_angle_d,
            top_angle_d = top_angle_dd,
            motor_torque= (signal - state.motor_torque) / self.params.motor_reaction_speed,
        )
            

    # Enforces limit conditions (i.e. top bouncing on floor)
    # Modifies state in-place
    def limit(self, state: SimulationState, signals: ControlSignals):
        max_angle = math.pi - math.acos(-self.params.wheel_rad / self.params.height)

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
    def step(self, state: SimulationState, signals: ControlSignals, dt: float) -> SimulationState:
        k1 = self.state_derivative(state, signals)
        k2 = self.state_derivative(state.apply_velocities(k1, dt / 2), signals)
        k3 = self.state_derivative(state.apply_velocities(k2, dt / 2), signals)
        k4 = self.state_derivative(state.apply_velocities(k3, dt), signals)
        res = state.apply_velocities(k1, dt/6).apply_velocities(k2, dt/3).apply_velocities(k3, dt/3).apply_velocities(k4, dt/6)
        self.limit(res, signals)
        return res
    
    def get_pos_vel(self, state: SimulationState):
        
        pos_x = self.params.height * np.sin(state.top_angle) 
        pos_z = self.params.height * np.cos(state.top_angle)
        pos_x_d = self.params.height * state.top_angle_d * np.cos(state.top_angle)
        pos_z_d = -self.params.height * state.top_angle_d * np.sin(state.top_angle)

        return PositionState(
                            top_center_pos_x = pos_x,
                            top_center_pos_z = pos_z,
                            top_center_pos_x_d = pos_x_d,
                            top_center_pos_z_d = pos_z_d)