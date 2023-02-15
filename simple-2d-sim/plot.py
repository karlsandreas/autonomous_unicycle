import numpy as np
import random

from pidcontroller import PIDController

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import time

from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter
import initials as init

INIT_STATE = init.INIT_STATE
DEFAULT_PARAMETERS = init.DEFAULT_PARAMETERS
# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = init.DEFAULT_REG
DEFAULT_KALMAN_GAIN = init.DEFAULT_KALMAN_GAIN



class plotter:
    def __init__(
        self,
        simulator: Simulator,
        init_state: SimulationState,
        reg: Regulator,
        var: float, #Noise variance 
        simtime: float, #Seconds of simultaion [S]
        dt: float) -> None: #Delta time in [S]
        
        self.sim = simulator
        self.init_state = self.state = init_state
        self.reg = reg
        self.current_signals = ControlSignals()

        self.filter = KalmanFilter(
            self.sim,
            init_state,
            SimulationState(),
            DEFAULT_KALMAN_GAIN,
        )
        self.var = var
        self.iterations = simtime/dt
        self.dt = dt
        self.sim_output = np.zeros(int(self.iterations))
        self.sim_output_w_noise = np.zeros(int(self.iterations))
        self.kalman_estimates = np.zeros(int(self.iterations))
        self.kalman_output = np.zeros(int(self.iterations))
        self.ticks = np.zeros(int(self.iterations))
        self.time = 0.0


    def step(self, dt: float, i) -> None:
        self.current_signals = self.reg(self.state, dt)
        self.time += dt

        self.state = self.sim.step(self.state, self.current_signals, dt)
        self.filter.step(dt, self.current_signals)
        kalman_output = self.filter.state #Save the kalman filter output 

        var_x = self.var
        var_z = self.var
        noise_x, noise_z = random.gauss(0, var_x**0.5), random.gauss(0, var_z**0.5)
        ax, az = self.sim.sensor_reading(self.state, self.current_signals)
        ax_w_noise, az_w_noise = ax + noise_x, az + noise_z

        self.filter.read_sensor((ax_w_noise, az_w_noise), (var_x, var_z), self.current_signals, dt)
        kalman_estimate = self.filter.state #Save kalman filter estimate
                
        self.sim_output[i] = self.state.top_angle

        self.kalman_estimates[i] = kalman_estimate.top_angle
        self.kalman_output[i] = kalman_output.top_angle
        self.ticks[i] = self.time

    #Loop the step function i iterations, 
    def run(self) -> None:
        for i in range(0, int(self.iterations)):
            self.step(self.dt, i)

    #Plots all signals save to class arrays
    def animate(self):
        
        plt.plot(self.ticks, self.sim_output, label= "Sim")
        plt.plot(self.ticks, self.kalman_estimates, label = "Kalman est")
        plt.plot(self.ticks, self.kalman_output, label = "Kalman out")


        # Format plot
        plt.ylabel('Top angle')
        plt.legend()
        plt.show()


p = plotter(
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REG,
    0.5,
    10,
    0.001
)

p.run()

p.animate()