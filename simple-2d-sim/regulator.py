from abc import ABC, abstractmethod

from sim import SimulationParameters, SimulationState_Pitch as SimulationState, ControlSignals

# Base class for all regulators.
# Should support a __call__, taking the simulation state and the dt, optionally changing the state of the regulator, giving a ControlSignals
class Regulator(ABC):
    def __init__(self, params: SimulationParameters):
        pass

    @abstractmethod
    def __call__(self, st: SimulationState, dt: float) -> ControlSignals:
        pass

class NullRegulator(Regulator):
    def __call__(self, st: SimulationState, dt: float) -> ControlSignals:
        return ControlSignals(motor_torque_signal=0)

class LookaheadSpeedRegulator(Regulator):
    def __init__(
        self,
        params: SimulationParameters,
        setpoint_x_d: float,
        settle_time_theta: float = 0.5,
        settle_time_x_d: float = 2.,
    ):
        super().__init__(params)
        self.A, self.B, self.C, self.D = params.abcd()
        self.E = self.A - self.C * self.B / self.D

        self.setpoint_x_d = setpoint_x_d

        self.last_delta_tau = 0.

    def dump_params(self):
        print(f"#define param_A {self.A}")
        print(f"#define param_B {self.B}")
        print(f"#define param_C {self.C}")
        print(f"#define param_D {self.D}")
        print(f"#define param_E {self.E}")

    def expected_theta_after(self, st: SimulationState, t: float) -> float:
        theta = st.top_angle
        theta_d = st.top_angle_d
        x = st.wheel_position
        x_d = st.wheel_position_d

        delta_tau = st.motor_torque + self.C / self.D * theta

        return theta + t * theta_d + t ** 2 / 2 * self.D * delta_tau

    def expected_x_after(self, st: SimulationState, t: float) -> float:
        theta = st.top_angle
        theta_d = st.top_angle_d
        x = st.wheel_position
        x_d = st.wheel_position_d

        delta_tau = st.motor_torque + self.C / self.D * theta

        return x + t * x_d + t**2 / 2 * (self.E * theta + self.B * delta_tau) + t**3/6 * (self.E * theta_d) + t**4 / 24 * (self.E * self.D * delta_tau)

    def expected_x_d_after(self, st: SimulationState, t: float) -> float:
        theta = st.top_angle
        theta_d = st.top_angle_d
        x = st.wheel_position
        x_d = st.wheel_position_d

        delta_tau = self.last_delta_tau # st.motor_torque + self.C / self.D * theta

        return x_d + t * (self.E * theta + self.B * delta_tau) + t**2/2 * (self.E * theta_d) + t**3 / 6 * (self.E * self.D * delta_tau)

    def stop_time_theta(self, st: SimulationState) -> float:
        return 2.0

    def stop_time_x_d(self, st: SimulationState) -> float:
        theta = st.top_angle
        theta_d = st.top_angle_d
        x = st.wheel_position
        x_d = st.wheel_position_d

        x_d_diff = abs(x_d - self.setpoint_x_d)
        return x_d_diff * 0.5 + 1.2

    def __call__(self, st: SimulationState, dt: float) -> float:
        theta = st.top_angle
        theta_d = st.top_angle_d
        x = st.wheel_position
        x_d = st.wheel_position_d

        t_theta = self.stop_time_theta(st)
        t_x_d = self.stop_time_x_d(st)

        # Want to zero theta
        delta_tau_theta = 1 / (t_theta ** 2 / 2 * self.D) * \
            (0 - theta - theta_d * t_theta)

        delta_tau_theta_d = 1 / (t_theta * self.D) * -theta_d

        delta_tau_x_d = 1 / (t_x_d * self.B + t_x_d ** 3 / 6 * self.E * self.D) * \
            (self.setpoint_x_d - x_d - t_x_d * self.E * theta - t_x_d ** 2 / 2 * self.E * theta_d)

        self.last_delta_tau = delta_tau_theta + delta_tau_theta_d + delta_tau_x_d

        tau = -self.C / self.D * theta + self.last_delta_tau

        return tau

