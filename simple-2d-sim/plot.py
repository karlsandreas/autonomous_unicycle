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
DEFAULT_KALMAN = init.DEFAULT_KALMAN
DEFAULT_REG = init.DEFAULT_REG
DEFAULT_KALMAN_GAIN = init.DEFAULT_KALMAN_GAIN



class plotter:
    def __init__(
        self,
        simulator: Simulator,
        init_state: SimulationState,
        reg: Regulator,
        kalman_filter: KalmanFilter,
        simtime: float, #Seconds of simultaion [S]
        dt: float) -> None: #Delta time in [S]
        
        self.sim = simulator
        self.init_state = self.state = init_state
        self.reg = reg
        self.current_signals = ControlSignals()
        
        self.filter = kalman_filter
        self.filter_state = init_state

        
        self.sensor_var = np.random.normal(0.0, 1, 3)
        self.sensor_var[0] *= 0.001  #Angular speed 
        self.sensor_var[1] *= 0.01  #Acceleration a_x
        self.sensor_var[2] *= 0.01  #Acceleration a_z

        self.iterations = simtime/dt
        self.dt = dt

        #Arrays to save plot values
        self.sim_output = np.zeros(int(self.iterations))
        self.sim_output_w_noise = np.zeros(int(self.iterations))
        self.sim_output_no_noise = np.zeros(int(self.iterations))
        self.kalman_output_d = np.zeros(int(self.iterations)) 
        self.kalman_output = np.zeros(int(self.iterations))
        self.sensor_a_n = np.zeros(int(self.iterations))
        self.ticks = np.zeros(int(self.iterations))
        self.time = 0.0


    def step(self, dt: float, i) -> None:
        self.current_signals = self.reg(self.state, dt)
        self.time += dt

        sensor_reading = self.sim.sensor_reading(self.state, self.current_signals)

        var_x = 0.2 #m/s^2
        var_z = 0.2 #m/s^2
        var_angle = 0.005 #rad/s 
        noise_x, noise_z, noise_angle = random.gauss(0, var_x**0.5), random.gauss(0,var_z**0.5), random.gauss(0, var_angle**0.5)
        sensor_var = np.array([noise_x, noise_z, noise_angle])

        sensor_reading += sensor_var
        
        a = (sensor_reading[1]**2 + sensor_reading[2]**2)**0.5

        top_angle_d = sensor_reading[0]

        #kalman_out = self.filter.predict(np.array([a_n, top_angle_d]).reshape(2,1))
        kalman_out = self.filter.predict(a)
        
        self.filter_state.top_angle = kalman_out[0][0]
        self.filter_state.top_angle_d = kalman_out[1][0]

        self.state = self.sim.step(self.state, self.current_signals, dt)
        self.filter_state = self.sim.step(self.filter_state, self.current_signals, dt)
        

        self.filter.update(top_angle_d)

        #Acceleration
        self.sensor_a_n[i] = a

        #Angular speed
        self.sim_output_no_noise[i] = self.state.top_angle
        self.sim_output_w_noise[i] = top_angle_d
        self.kalman_output_d[i] = kalman_out[1][0]

        #Angles
        self.sim_output[i] = self.state.top_angle
        self.kalman_output[i] = kalman_out[0][0]
        self.ticks[i] = self.time

    #Loop the step function i iterations, 
    def run(self) -> None:
        for i in range(0, int(self.iterations)):
            self.step(self.dt, i)

    #Plots all signals save to class arrays
    def animate(self):
        fig, axs = plt.subplots(3)

        axs[2].plot(self.ticks, self.sensor_a_n, label= "Accleration with noise")
        
        axs[1].plot(self.ticks, self.sim_output_no_noise, label= "Sim angle_d")
        axs[1].plot(self.ticks, self.sim_output_w_noise, label= "Sim angle_d noise")
        axs[1].plot(self.ticks, self.kalman_output_d, label= "Kalman angle_d ")
        axs[0].plot(self.ticks, self.sim_output, label= "Sim")        
        axs[0].plot(self.ticks, self.kalman_output, label = "Kalman out")


        # Format plot
        #fig.ylabel('Top angle')
        fig.legend()
        plt.show()


p = plotter(
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REG,
    DEFAULT_KALMAN,
    30,
    0.001
)

p.run()

p.animate()