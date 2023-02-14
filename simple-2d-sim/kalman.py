from typing import Tuple

from sim import Simulator, SimulationState, ControlSignals

# Assumes linear model, instant motor torque
# var = variance
class KalmanFilter:
    def __init__(
        self,
        sim: Simulator,
        state: SimulationState,
        probs: SimulationState,
        gain: float, # per second
    ):
        self.sim = sim
        self.state = state
        self.var = probs
        self.gain = gain

    def step(self, dt: float, sig: ControlSignals, sig_prob=ControlSignals()):
        A, B, C, D = self.sim.params.abcd()

        wheel_position_dd = A * self.state.top_angle + B * sig.motor_torque_signal
        wheel_position_var_dd = A * self.var.top_angle + B * sig_prob.motor_torque_signal

        top_angle_dd = C * self.state.top_angle + D * sig.motor_torque_signal
        top_angle_var_dd = C * self.var.top_angle + D * sig_prob.motor_torque_signal

        new_state = SimulationState(
            wheel_position=self.state.wheel_position + dt * self.state.wheel_position_d,
            wheel_position_d=self.state.wheel_position_d + dt * wheel_position_dd,
            top_angle=self.state.top_angle + dt * self.state.top_angle_d,
            top_angle_d=self.state.top_angle_d + dt * top_angle_dd,
            motor_torque=sig.motor_torque_signal,
        )

        new_var = SimulationState(
            wheel_position=self.var.wheel_position + dt * self.var.wheel_position_d,
            wheel_position_d=self.var.wheel_position_d + dt * wheel_position_var_dd,
            top_angle=self.var.top_angle + dt * self.var.top_angle_d,
            top_angle_d=self.var.top_angle_d + dt * top_angle_var_dd,
            motor_torque=sig_prob.motor_torque_signal,
        )

        self.state = new_state
        self.var = new_var

    def read_sensor(
        self,
        sensor_reading: Tuple[float, float],
        sensor_variance: Tuple[float, float],
        sig: ControlSignals,

        reading_time: float,
    ):
        A, B, C, D = self.sim.params.abcd()
        R = self.sim.params.sensor_position

        expected_reading = (
            (A + R * C) * self.state.top_angle + (B + D * R) * sig.motor_torque_signal,
            (A * self.state.top_angle) * self.state.top_angle - (R * self.state.top_angle_d) * self.state.top_angle_d
        )

        diff = sensor_reading[0] - expected_reading[0], sensor_reading[1] - expected_reading[1]

        gain = self.gain * reading_time

        state_delta = SimulationState(
            wheel_position=self.state.wheel_position,
            wheel_position_d=self.state.wheel_position_d,
            top_angle=(A + R * C) * diff[0] + (A * self.state.top_angle) * diff[1],
            top_angle_d=-R * self.state.top_angle_d * diff[1],
        )

        var_delta = SimulationState(
            wheel_position=self.var.wheel_position,
            wheel_position_d=self.var.wheel_position_d,
            top_angle=(A + R * C) * sensor_variance[0] + (A * self.state.top_angle) * sensor_variance[1],
            top_angle_d=-R * self.state.top_angle_d * sensor_variance[1],
        )

        self.state = SimulationState().apply_velocities(self.state, 1 - gain).apply_velocities(state_delta, gain)
        self.var = SimulationState().apply_velocities(self.var, 1 - gain).apply_velocities(var_delta, gain)
