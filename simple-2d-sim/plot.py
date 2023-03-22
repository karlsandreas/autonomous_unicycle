import numpy as np
from pathlib import Path
import random

import matplotlib.pyplot as plt

from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter
import initials as init
from copy import deepcopy

#Imports for c code
from ctypes import *
so_file = Path("STM32/regulator.so")

reg = CDLL(so_file.resolve())

so_file_filter = Path("STM32/kalman_filter.so")
c_kalman = CDLL(so_file_filter.resolve())

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



class PlotValuesForOneGraph:
    def __init__(self, itterations: int, labels: list):
        self.data = {}
        self.labels = labels

        for label in labels:
            self.data[label] = np.zeros(itterations)


        self.ticks = np.linspace(0, itterations, num=itterations)
    
    def add(self, values: list, i):
        j = 0
        for label in self.labels:
            self.data[label].data[i] = values[j]
            j += 1
        
    def get(self):

        return [self.ticks, self.data]


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
        
        self.reg_version = "Python" #C
        self.filter_version = "Python" #C

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
        self.var_rpm = 1 #rpm
        self.var_angle_d = 0.01 #rad/s 
        self.sensor_var = np.zeros(3) 
        self.sensor_var[0] = random.gauss(0, var_x)
        self.sensor_var[1] = random.gauss(0, var_z)
        self.sensor_var[2] = random.gauss(0, self.var_angle_d)

        self.iterations = int(simtime/dt)
        self.dt = dt

        self.angle = PlotValuesForOneGraph(self.iterations,["Angle clean", "Angle kalman"])
        self.angle_d = PlotValuesForOneGraph(self.iterations,["Angle_d clean","Angle_d noise", "Angle_d kalman"])
        self.wheel_d = PlotValuesForOneGraph(self.iterations,["Wheel clean","Wheel noise", "Wheel kalman"])
        self.p_vals = PlotValuesForOneGraph(self.iterations, ["P11","P12", "P21", "P22"])
        self.wheel_rpm = PlotValuesForOneGraph(self.iterations, ["RPM clean","RPM noise", "RPM kalman"])


        self.time = 0.0

        self.top_angle_d = 0.0
        self.a = 0.0

        self.saved_data = True

    def step_reg_filter(self, dt: float, i) -> None:

        if self.filter_version == "C":
            ######## Kalman filter in C ##########
            #Update the Qs since they depend on dt
            self.c_Qs.contents.m11 = c_float(2.0 * (dt**4)/4) 
            self.c_Qs.contents.m12 = c_float(2.0 * (dt**3)/2)
            self.c_Qs.contents.m21 = c_float(2.0 * (dt**3)/2)
            self.c_Qs.contents.m22 = c_float(2.0 * dt**2)

            #self.c_state.contents.x3 = self.filter_state.wheel_position #Update states in pointer since we are not mesuring them
            #self.c_state.contents.x4 = self.filter_state.wheel_position_d
        
        if self.filter_version == "C":
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

        if self.filter_version == "Python":
            filter_states = self.filter.predict()
            self.filter_state.top_angle = filter_states[0][0]
            self.filter_state.top_angle_d = filter_states[1][0]
            #self.filter_state.wheel_position_d = self.filter_wheel.predict()


        #Angular speed
        
        self.angle_d.add([self.state.top_angle_d, self.top_angle_d, self.filter_state.top_angle_d],i)

        self.angle.add([self.state.top_angle, self.filter_state.top_angle],i) 
        
        self.wheel_d.add([self.state.wheel_position_d, self.filter_state.wheel_position_d],i)

        
        #Angles

        
        self.time += dt

        if self.reg_version == "C":
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
        
        if self.reg_version == "Python":
            ############### Regulator in python ########
            self.current_signals = self.reg(self.filter_state, dt )
            ############################################

        ############ Integration step ###############
        self.filter_state = deepcopy(self.state)

        ########### Sensor reading and noise #######
        sensor_reading = self.sim.sensor_reading(self.filter_state, self.current_signals)
        #sensor_reading = self.sim.sensor_reading(self.state, self.current_signals)
        
        var_x = 0.0004 #m/s^2
        var_z = 0.0004 #m/s^2
        var_angle_d = 0.001 #rad/s 
        noise_x, noise_z, noise_angle = random.gauss(0, var_x), random.gauss(0, var_z), random.gauss(0, var_angle_d)

        a_x = sensor_reading[1] + noise_x
        a_z = sensor_reading[2] + noise_z
        self.a = (a_x**2 + a_z**2)**0.5


        top_angle_d = sensor_reading[0] + noise_angle
        self.top_angle_d = top_angle_d

        self.sensor_reading = top_angle_d #Copy to class for info tab
        
        x_d = self.filter_state.wheel_position 
        #self.wheel_d_sim[i] = x_d
        x_d_noise = random.gauss(0, 0.1)
        x_d_w_noise = x_d + x_d_noise
        #self.wheel_d_noise[i] = x_d_w_noise

        if self.filter_version == "C":
            ##### C Kalman filter #####
            c_kalman.pitch_kalman_filter_update(c_float(top_angle_d), c_float(dt), self.c_state, self.c_Qs)
            #c_kalman.wheel_velocity_kalman_filter_update(c_float(x_d_w_noise), c_float(dt));
        
        if self.filter_version == "Python":
            self.filter.update(top_angle_d)
        
            self.p_vals.add([self.filter.P[0][0],self.filter.P[0][1],self.filter.P[1][0],self.filter.P[1][1]],i)
            

            #self.filter_wheel.update(x_d_w_noise)

    def step(self, dt: float, i) -> None:
        self.state = self.sim.step(self.state, self.current_signals, dt)
        
    #Loop the step function i iterations, 
    def run(self) -> None:

        if self.saved_data:
            saved_states = self.load_saved_states("sensor_data/sim_all_states_22_03_dt_0_001.npy")
            for i in range(0, self.iterations): 
                self.step_saved_states(self.dt, i, saved_states)
        else:
            for i in range(0, int(self.iterations)):
                if (i > self.iterations/2):
                    self.reg.setpoint_x_d = 1

                self.step_reg_filter(self.dt,i)
                self.step(self.dt, i)
            #np.save('./sensor_data/sim_wheel_pos_d', self.x_ds)

    def update_state_dict_to_simstates(self, state_dict, i):
        self.state.top_angle = state_dict["top_angle"].data[i] 
        self.state.top_angle_d = state_dict["top_angle_d"].data[i]
        self.state.wheel_position = state_dict["wheel_position"].data[i]
        self.state.wheel_position_d = state_dict["wheel_position_d"].data[i]

            
    def step_saved_states(self, dt, i, state_dict):
        top_angle = state_dict["top_angle"].data[i]
        top_angle_d = state_dict["top_angle_d"].data[i] 
        wheel_d = state_dict["wheel_position_d"].data[i]
        wheel_rpm = (wheel_d / DEFAULT_PARAMETERS.wheel_rad) * (30 / np.pi)


        if self.filter_version == "Python":
            filter_states = self.filter.predict()
            kalman_top_angle = filter_states[0][0]
            kalman_top_angle_d = filter_states[1][0]
            kalman_wheel = filter_states[2][0]
            kalman_wheel_d = filter_states[3][0]

        top_angle_d_noise = self.add_noise(self.var_angle_d, top_angle_d)
        
        wheel_rpm_noise = self.add_noise(self.var_rpm, wheel_rpm)

        
        


        if self.filter_version == "Python":
            sensor_reading = np.array([top_angle_d_noise,wheel_rpm_noise]).reshape(2,1)
            self.filter.update(sensor_reading)

            self.p_vals.add([self.filter.P[0][0],self.filter.P[0][1],self.filter.P[1][0],self.filter.P[1][1]],i)
            
            #self.filter_wheel.update(wheel_rpm_noise)
        
        self.angle_d.add([top_angle_d, top_angle_d_noise, kalman_top_angle_d],i)
        
        self.angle.add([top_angle, kalman_top_angle],i) 

        self.wheel_d.add([wheel_d, kalman_wheel_d, kalman_wheel_d],i)

        self.wheel_rpm.add([wheel_rpm, wheel_rpm_noise, kalman_wheel_d *30/np.pi / DEFAULT_PARAMETERS.wheel_rad],i)


    def add_noise(self, variance, input: float):
        noise = random.gauss(0, variance)
        return input + noise

    def load_saved_states(self, path):
        npfile = np.load(path)
        nptransposed = np.transpose(npfile)
        saved_states = {}
        saved_states["top_angle"] = nptransposed[0]
        saved_states["top_angle_d"] = nptransposed[1]
        saved_states["wheel_position"] = nptransposed[2]
        saved_states["wheel_position_d"] = nptransposed[3]
        return saved_states


    #Plots all graphs
    def draw_plots(self):
        plots = []
        for name, var in self.__dict__.items():
            if isinstance(var,PlotValuesForOneGraph):
                plots.append(var)
        
        fig,axs = plt.subplots(len(plots))

        for i in range(len(plots)):
            plot_obj = plots[i]
            data = plot_obj.get()
            for lab in plot_obj.labels:
                vals = data[1]
                axs[i].plot(data[0],vals[lab], label = lab) 

        fig.legend()
        plt.xlabel("Time [s]")
        axs[0].set_ylabel("Rad")
        axs[1].set_ylabel("Rad/s")
        #axs[2].set_ylabel("m/s^2")
        plt.show()



p = Plotter(
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REG,
    DEFAULT_KALMAN,
    DEFAULT_KALMAN_WHEEL,
    30,
    0.001
)


p.run()

p.draw_plots()





def save_sensor_data(sim: Simulator, 
                     state: SimulationState, 
                     reg: Regulator, 
                     dt: float, 
                     simtime: float, 
                     unitstep_at: float, #percent of simtime
                     setpoint_at_step: float, #Setpoint when unitstep
                     save_path):
    
    iterartions = int(simtime/dt)
    unitstep_i = iterartions * unitstep_at
    saved_states = []
    for i in range(0, iterartions):
        if unitstep_i == i:
            reg.setpoint_x_d = setpoint_at_step
    
        control_signal = reg(state, dt)
        state = sim.step(state, control_signal, dt)    
        saved_states.append(state.get_array())

    np_saved_states = np.array(saved_states)

    np.save(save_path, np_saved_states)


#save_sensor_data(Simulator(DEFAULT_PARAMETERS),INIT_STATE,DEFAULT_REG,0.001,30,0.5,1.0,"sensor_data/sim_all_states_22_03_dt_0_001")

