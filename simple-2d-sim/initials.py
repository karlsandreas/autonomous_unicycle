from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter


INIT_STATE = SimulationState(
    wheel_position = 0,
    wheel_position_d = 0,
    top_angle = 0,
    top_angle_d = 0,
    motor_torque = 0,
)

DEFAULT_PARAMETERS = SimulationParameters(
    wheel_rad = 0.28,
    wheel_mass = 20,
    top_height = 0.8,
    top_mass = 4,
    motor_reaction_speed = 0.1,
    sensor_position=1.0,
)

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = LookaheadSpeedRegulator(
    params=DEFAULT_PARAMETERS,
    setpoint_x_d=1.,
)

DEFAULT_KALMAN_GAIN = 0.5