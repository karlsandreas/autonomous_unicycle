import numpy as np
import random

import matplotlib.pyplot as plt

import initials as init

#Imports for c code
from ctypes import *
so_file = "C:/Users/ante_/Documents/Kandidatarbete/autonomous_unicycle/STM32/regulator.so"
reg = CDLL(so_file)

so_file_filter = "C:/Users/ante_/Documents/Kandidatarbete/autonomous_unicycle/STM32/kalman_filter.so"
c_kalman = CDLL(so_file_filter)

INIT_STATE = init.INIT_STATE
DEFAULT_PARAMETERS = init.DEFAULT_PARAMETERS
# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_KALMAN = init.DEFAULT_KALMAN
DEFAULT_REG = init.DEFAULT_REG
DEFAULT_KALMAN_GAIN = init.DEFAULT_KALMAN_GAIN

npfile = np.load('sensor_data/last-run.npz')
kalman_data = npfile['kalman_data']
sensor_data = npfile['sensor_data']

sim_wheel_sensor = np.load('sensor_data/sim_wheel_pos_d.npy')
#sim_wheel_sensor = sim_sensor_file['x_ds']

print(sensor_data.shape)

steps = sim_wheel_sensor.size
sensor_with_noise = np.random.normal(0, 0.10,size=(sim_wheel_sensor.size,1))
sin_range = np.sin(np.linspace(-np.pi, np.pi,10000)) #.resize(201,1))
#sensor_low_freq_noise = np.repeat(sin_range, 3)

#a = np.add(sensor_low_freq_noise, sensor_with_noise)
sim_wheel_sensor_noise = np.add(sim_wheel_sensor.reshape(sim_wheel_sensor.size,1), sensor_with_noise)

kalman_out = np.zeros(steps)
c_kalman_vel = c_kalman.wheel_velocity_kalman_filter_predict
c_kalman_vel.restype = c_float  # Set output type from c code

for i in range(0,steps):
    dt = 0.001
    kalman_out[i] = c_kalman_vel(c_float(dt))
    
    c_kalman.wheel_velocity_kalman_filter_update(c_float(sim_wheel_sensor_noise[i]), c_float(dt));

time_axis = np.linspace(0, sim_wheel_sensor.size/0.01, sim_wheel_sensor.size)

fig, axs = plt.subplots(2)
axs[0].plot(time_axis, sim_wheel_sensor, time_axis, kalman_out, label= "Accleration with noise")
axs[1].plot(time_axis, sim_wheel_sensor_noise,time_axis, kalman_out, label= "Accleration with noise")
#%matplotlib qt
fig.legend()
plt.show()