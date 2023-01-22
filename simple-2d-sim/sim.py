from __future__ import annotations

import numpy as np
import math

BOUNCE_FACTOR = 0.3

# Represents the state of the simulation
# Has fields params for parameters which aren't affected by the simulation (both parameters such as lengths of static parts, and parameters which can be changed by the user/control system)
# and field state which are all the time-dependent variables affected by the simulation
#
# Both of these are represented as numpy arrays for simpler batch-wise operations (such as element-wise addition etc.)
# but should probably be represented by classes in more idiomatic code
#
# deriv gives the derivative of state with respect to time
# limit is called after a simulation step has been made, to make sure discrete boundary conditions are handled
# (currently only does collision with floor checking)

class SimulationState:
    def __init__(
        self,
        params: np.ndarray, # (torque_wanted, g, r0, R1, m0, m1, motor_tau)
        state: np.ndarray, # (x, x_d, θ1, θ1_d, torque),
    ):
        self.params = params
        self.state = state

    def deriv(self) -> np.ndarray:
        torque_wanted, g, r0, R1, m0, m1, motor_tau = self.params
        x, x_d, θ1, θ1_d, torque = self.state

        F_g = m1 * g
        F_g1 = F_g * np.cos(θ1)
        F_g2 = F_g * np.sin(θ1)
        F_g4 = F_g2 * np.cos(θ1)

        return np.array([
            x_d,
            F_g4 / m0 - torque / (r0 * m0),
            θ1_d,
            -F_g1 / (R1 * m1) - torque / (R1 * m1),
            (torque_wanted - torque) / motor_tau,
        ])

    def apply_velocities(self, vels: np.ndarray) -> SimulationState:
        return SimulationState(
            self.params,
            self.state + vels,
        )

    def limit(self) -> SimulationState:
        torque_wanted, g, r0, R1, m0, m1, motor_tau = self.params
        x, x_d, θ1, θ1_d, torque = self.state

        if R1 < r0:
            return self

        min_theta1 = math.asin(-r0 / R1)
        if θ1 < min_theta1:
            return SimulationState(
                self.params,
                np.array([x, x_d * BOUNCE_FACTOR, min_theta1, -BOUNCE_FACTOR * θ1_d, torque]),
            )

        max_theta1 = math.pi + math.asin(r0 / R1)
        if θ1 > max_theta1:
            return SimulationState(
                self.params,
                np.array([x, x_d * BOUNCE_FACTOR, max_theta1, -BOUNCE_FACTOR * θ1_d, torque]),
            )

        return self


# Simple class to apply the classic Runge-Kutta (4) integration method to the state
class RungeKuttaIntegrator:
    def __init__(self) -> None:
        pass

    def step(self, system: SimulationState, h: float) -> SimulationState:
        k1 = system.deriv()
        k2 = system.apply_velocities(h / 2 * k1).deriv()
        k3 = system.apply_velocities(h / 2 * k2).deriv()
        k4 = system.apply_velocities(h * k3).deriv()

        return system.apply_velocities(h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

