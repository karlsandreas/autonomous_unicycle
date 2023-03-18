import numpy as np
import random

import matplotlib.pyplot as plt

from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter
import initials as init
from copy import deepcopy

#Imports for c code
from ctypes import *
so_file = "./STM32/regulator.so"
reg = CDLL(so_file)

so_file_filter = "./STM32/kalman_filter.so"
c_kalman = CDLL(so_file_filter)

INIT_STATE = init.INIT_STATE
DEFAULT_PARAMETERS = init.DEFAULT_PARAMETERS
# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_KALMAN = init.DEFAULT_KALMAN
DEFAULT_REG = init.DEFAULT_REG
DEFAULT_KALMAN_GAIN = init.DEFAULT_KALMAN_GAIN
DEFAULT_KALMAN_WHEEL = init.DEFAULT_KALMAN_WHEEL



class States(Structure):
    _fields_ = [("x1", c_float),
                ("x2", c_float),
                ("x3", c_float),
                ("x4", c_float)]

class Matrix(Structure):
    _fields_ = [("m11", c_float),
                ("m12", c_float),
                ("m21", c_float),
                ("m22", c_float)]

c_code = False

class Plotter:
    def __init__(
        self,
        simulator: Simulator,
        init_state: SimulationState,
        reg: Regulator,
        kalman_filter: KalmanFilter,
        kalman_filter_wheel: KalmanFilter,
        simtime: float, #Seconds of simultaion [S]
        dt: float) -> None: #Delta time in [S]
        
        self.sim = simulator
        self.init_state = self.state = init_state
        self.reg = reg
        self.current_signals = ControlSignals()
        
        self.filter = kalman_filter 
        self.filter_wheel = kalman_filter_wheel
        self.filter_state = init_state

        self.c_state = pointer(States(0.0, 0.0, 0.0, 0.0))
        dt = init.dt
        self.c_Qs = pointer(Matrix(0.05*dt**2, 0.05*dt, 0.05*dt, 0.05))

        var_x = 0.0004 #m/s^2
        var_z = 0.0004 #m/s^2
        var_angle = 0.01 #rad/s 
        self.sensor_var = np.zeros(3) 
        self.sensor_var[0] = random.gauss(0, var_x)
        self.sensor_var[1] = random.gauss(0, var_z)
        self.sensor_var[2] = random.gauss(0, var_angle)

        self.iterations = int(simtime/dt)
        self.dt = dt

        #Arrays to save plot values

        self.sim_output = np.zeros(self.iterations)
        self.sim_output_w_noise = np.zeros(self.iterations)
        self.sim_output_no_noise = np.zeros(self.iterations)
        self.kalman_output_d = np.zeros(self.iterations) 
        self.kalman_output = np.zeros(self.iterations)
        self.sensor_a_n = np.zeros(self.iterations)
        self.ticks = np.zeros(self.iterations)
        self.time = 0.0

        self.top_angle_d = 0.0
        self.a = 0.0

        self.wheel_d = np.zeros(self.iterations)
        self.wheel_d_noise = np.zeros(self.iterations)
        self.wheel_d_sim = np.zeros(self.iterations)
    def step_reg_filter(self, dt: float, i) -> None:

        ######## Kalman filter in C ##########
        #Update the Qs since they depend on dt
        self.c_Qs.contents.m11 = c_float(2.0 * (dt**4)/4) 
        self.c_Qs.contents.m12 = c_float(2.0 * (dt**3)/2)
        self.c_Qs.contents.m21 = c_float(2.0 * (dt**3)/2)
        self.c_Qs.contents.m22 = c_float(2.0 * dt**2)

        #self.c_state.contents.x3 = self.filter_state.wheel_position #Update states in pointer since we are not mesuring them
        #self.c_state.contents.x4 = self.filter_state.wheel_position_d
        
        if c_code:
            c_kalman.pitch_kalman_filter_predict(c_float(self.a), c_float(dt), self.c_state, self.c_Qs)
            self.filter_state.top_angle = self.c_state.contents.x1
            self.filter_state.top_angle_d = self.c_state.contents.x2

            #Wheel filter
            #c_kalman_vel = c_kalman.wheel_velocity_kalman_filter_predict
            #c_kalman_vel.restype = c_float  # Set output type from c code
            #self.filter_state.wheel_position_d = c_kalman_vel(c_float(dt))
            c_kalman_simple = c_kalman.simple_wheel_filter
            c_kalman_simple.restype = c_float

            self.filter_state.wheel_position_d = c_kalman_simple(c_float(self.wheel_d_noise[i-1]), c_float(dt))

        else:
            filter_states = self.filter.predict()
            self.filter_state.top_angle = filter_states[0][0]
            self.filter_state.top_angle_d = filter_states[1][0]
            #self.filter_state.wheel_position_d = self.filter_wheel.predict()
        
        self.wheel_d[i] = self.filter_state.wheel_position_d


        #Angular speed
        self.sim_output_no_noise[i] = self.state.top_angle_d
        self.sim_output_w_noise[i] = self.top_angle_d
        self.kalman_output_d[i] = self.filter_state.top_angle_d

        #Angles
        self.sim_output[i] = self.state.top_angle
        self.kalman_output[i] = self.filter_state.top_angle
        self.time += dt
        self.ticks[i] = self.time

        ############## Regulator in c ##############
        ## Update params in regulator.h if changed in sim
        c_regulator = reg.LookaheadSpeedRegulator
        c_regulator.restype = c_float  # Set output type from c code 

        
        
        self.current_signals = ControlSignals(c_regulator(c_float(self.reg.setpoint_x_d), 
                                                c_float(self.filter_state.top_angle), 
                                                c_float(self.filter_state.top_angle_d),
                                                c_float(self.filter_state.wheel_position_d),
                                                c_float(dt)))
        #############################################

        ############ Integration step ###############
        self.filter_state = deepcopy(self.state)

        ########### Sensor reading and noise #######
        sensor_reading = self.sim.sensor_reading(self.filter_state, self.current_signals)
        #sensor_reading = self.sim.sensor_reading(self.state, self.current_signals)
        
        var_x = 0.0004 #m/s^2
        var_z = 0.0004 #m/s^2
        var_angle = 0.01 #rad/s 
        noise_x, noise_z, noise_angle = random.gauss(0, var_x), random.gauss(0, var_z), random.gauss(0, var_angle)

        a_x = sensor_reading[1] + noise_x
        a_z = sensor_reading[2] + noise_z
        self.a = (a_x**2 + a_z**2)**0.5

        #Acceleration
        self.sensor_a_n[i] = self.a

        top_angle_d = sensor_reading[0] + noise_angle
        self.top_angle_d = top_angle_d

        self.sensor_reading = top_angle_d #Copy to class for info tab
        
        x_d = self.filter_state.wheel_position 
        self.wheel_d_sim[i] = x_d
        x_d_noise = random.gauss(0, 0.1)
        x_d_w_noise = x_d + x_d_noise
        self.wheel_d_noise[i] = x_d_w_noise

        ##### C Kalman filter #####
        if c_code:
            c_kalman.pitch_kalman_filter_update(c_float(top_angle_d), c_float(dt), self.c_state, self.c_Qs)
            #c_kalman.wheel_velocity_kalman_filter_update(c_float(x_d_w_noise), c_float(dt));
        else:
            self.filter.update(top_angle_d)
            self.filter_wheel.update(x_d_w_noise)

    def step(self, dt: float, i) -> None:
        self.state = self.sim.step(self.state, self.current_signals, dt)
        
    #Loop the step function i iterations, 
    def run(self) -> None:

        
        for i in range(0, int(self.iterations)):
            if (i > self.iterations/2):
                self.reg.setpoint_x_d = 1

            self.step_reg_filter(self.dt,i)
            self.step(self.dt, i)
        #np.save('./sensor_data/sim_wheel_pos_d', self.x_ds)
        
    #Plots all signals save to class arrays
    def animate(self):
        fig, axs = plt.subplots(4)
        axs[0].plot(self.ticks, self.sim_output, label= "Sim")        
        axs[0].plot(self.ticks, self.kalman_output, label = "Kalman out")

        

        axs[1].plot(self.ticks, self.sim_output_no_noise, label= "Sim angle_d")
        axs[1].plot(self.ticks, self.sim_output_w_noise, label= "Sim angle_d noise")
        axs[1].plot(self.ticks, self.kalman_output_d, label= "Kalman angle_d ")
        
        axs[2].plot(self.ticks, self.sensor_a_n, label= "Accleration with noise")
        
        axs[3].plot(self.ticks, self.wheel_d, label= "Wheel vel kalman")
        axs[3].plot(self.ticks, self.wheel_d_noise, label= "Wheel vel with noise")
        axs[3].plot(self.ticks, self.wheel_d_sim, label= "Wheel vel sim")


        # Format plot
        fig.legend()
        plt.xlabel("Time [s]")
        axs[0].set_ylabel("Rad")
        axs[1].set_ylabel("Rad/s")
        axs[2].set_ylabel("m/s^2")
        plt.show()


p = Plotter(
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REG,
    DEFAULT_KALMAN,
    DEFAULT_KALMAN_WHEEL,
    20,
    0.001
)

p.run()

p.animate()