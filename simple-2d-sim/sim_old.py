from __future__ import annotations
from typing import Tuple

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

    def abcd(self) -> Tuple[float, float, float, float]:
        A = -self.g * self.top_mass / self.wheel_mass
        B = -1 / (self.wheel_rad * self.wheel_mass)
        C = self.g / self.top_height
        D = 1 / (self.top_height ** 2 * self.top_mass)
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
        gravity_top = self.params.g * self.params.top_mass # F_g = mg
        gravity_top_norm = gravity_top * np.sin(state.top_angle)
        gravity_top_parallel = gravity_top * np.cos(state.top_angle) # Positive = towards wheel
        force_on_wheel_center = -gravity_top_parallel * np.sin(state.top_angle) # Positive = rightwards

        wheel_inertia = self.params.wheel_mass * self.params.wheel_rad ** 2
        # Ignoring the indirect static force on wheel center from top
        wheel_angle_dd = state.motor_torque / wheel_inertia

        # Inertia around wheel center
        top_inertia = self.params.top_mass * self.params.top_height ** 2
        top_angle_dd = \
            gravity_top_norm * self.params.top_height / top_inertia \
            + state.motor_torque / top_inertia


        # All entries are derivatives
        return SimulationState(
            wheel_position = state.wheel_position_d,
            wheel_position_d = \
                force_on_wheel_center / self.params.wheel_mass \
                - wheel_angle_dd * self.params.wheel_rad,

            top_angle = state.top_angle_d,
            top_angle_d = top_angle_dd,

            motor_torque = (signals.motor_torque_signal - state.motor_torque) / self.params.motor_reaction_speed,
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