from typing import Tuple, Optional

import faulthandler


faulthandler.enable()

import pygame
import numpy as np
import random
from pathlib import Path
from copy import deepcopy

from pidcontroller import PIDController

import math

import time

from sim import SimulationState_Pitch, SimulationState_Roll, SimulationState, SimulationParameters, ControlSignals, Simulator_Pitch, Simulator_Roll, ms_to_rpm, rpm_to_ms
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter

from resim import Resimulator_Pitch

from fmt import fmt_unit
import initials as init
#Imports for c code
from ctypes import *
so_file = Path("STM32/regulator.so")
reg = CDLL(so_file.resolve())

so_file_filter = Path("STM32/kalman_filter.so")
c_kalman = CDLL(so_file_filter.resolve())

# Parameters for rendering
BORDER = 4

LINE_COL = (64, 64, 64)
LINE_COL0 = (100, 100, 100)

Color = Tuple[int, int, int]


class States(Structure):
    _fields_ = [("x1", c_float),
                ("x2", c_float),
                ("x3", c_float),
                ("x4", c_float),
                ("x5", c_float),
                ("x6", c_float)]

class Matrix(Structure):
    _fields_ = [("m11", c_float),
                ("m12", c_float),
                ("m21", c_float),
                ("m22", c_float)]
    
class Covariances(Structure):
    _fields_ = [("pitch", Matrix),
                ("roll", Matrix),
                ("wheel", Matrix)]

class R_error(Structure):
    _fields_ = [("pitch", c_float),
                ("roll", c_float),
                ("wheel", c_float)]


class SimRenderOptions:
    def __init__(
        self,
        wheel_color: Color = (200, 200, 200),
        spoke_color: Color = (150, 150, 150),

        torque_color: Color = (255, 0, 0),
        setpoint_color: Color = (0, 255, 0),
        expected_color: Color = (0, 255, 255),

        sensor_color: Color = (255, 0, 128),
        sensor_measure_color: Color = (255, 0, 255),

        outside_thickness: float = 0.2,
        inner_size: float = 0.6,
        inner_thickness: float = 0.2,
        n_spokes: int = 20,
        spoke_size: float = 0.1,

        torque_size: float = 0.5,

        draw_sensor: bool = True,
        draw_torque: bool = True,
    ):
        self.wheel_color = wheel_color
        self.spoke_color = spoke_color
        self.torque_color = torque_color
        self.setpoint_color = setpoint_color
        self.expected_color = expected_color
        self.sensor_color = sensor_color
        self.sensor_measure_color = sensor_measure_color
        self.outside_thickness = outside_thickness
        self.inner_size = inner_size
        self.inner_thickness = inner_thickness
        self.n_spokes = n_spokes
        self.spoke_size = spoke_size
        self.torque_size = torque_size

        self.draw_sensor = draw_sensor
        self.draw_torque = draw_torque

class SimRenderOptions_2:
    def __init__(
        self,
        wheel_color: Color = (100, 100, 200),
        spoke_color: Color = (100, 100, 150),

        torque_color: Color = (0, 255, 0),
        setpoint_color: Color = (0, 0, 255),
        expected_color: Color = (0, 255, 0),

        sensor_color: Color = (255, 0, 128),
        sensor_measure_color: Color = (255, 0, 255),

        outside_thickness: float = 0.2,
        inner_size: float = 0.6,
        inner_thickness: float = 0.2,
        n_spokes: int = 20,
        spoke_size: float = 0.1,

        torque_size: float = 0.5,

        draw_sensor: bool = True,
        draw_torque: bool = True,
    ):
        self.wheel_color = wheel_color
        self.spoke_color = spoke_color
        self.torque_color = torque_color
        self.setpoint_color = setpoint_color
        self.expected_color = expected_color
        self.sensor_color = sensor_color
        self.sensor_measure_color = sensor_measure_color
        self.outside_thickness = outside_thickness
        self.inner_size = inner_size
        self.inner_thickness = inner_thickness
        self.n_spokes = n_spokes
        self.spoke_size = spoke_size
        self.torque_size = torque_size

        self.draw_sensor = draw_sensor
        self.draw_torque = draw_torque


#INFO_FONT = "ShareTech.ttf", 30 # path, size
INFO_FONT = Path("simple-2d-sim/ShareTech.ttf").resolve(), 30

# all relative to wheel diameter

CAMERA_TAU = 0.1
ZOOM_TAU = 0.05

MIN_DT = 0.001

# Translating from/to pixel-space to/from unit-space
class ScreenSpaceTranslator:
    def __init__(
        self,
        pixels_per_unit: float,
        view_center: np.ndarray,
        screen_size: Tuple[int, int],
    ):
        self.pixels_per_unit = pixels_per_unit
        self.view_center = view_center
        self.screen_center = np.array([float(screen_size[0]), float(screen_size[1])]) / 2

    def set_screen_size(self, screen_size: Tuple[int, int]) -> None:
        self.screen_center = np.array([float(screen_size[0]), float(screen_size[1])]) / 2

    def screen2unit(self, screen: Tuple[float, float]) -> np.ndarray:
        screen_np = np.array([screen[0], screen[1]])
        return (screen - self.screen_center) / self.pixels_per_unit  * np.array([1, -1])+ self.view_center

    def unit2screen(self, unit: np.ndarray) -> Tuple[float, float]:
        sx, sy = (unit - self.view_center) * self.pixels_per_unit * np.array([1, -1]) + self.screen_center
        return sx, sy

    def unit2screen_(self, ux: float, uy: float) -> Tuple[float, float]:
        return self.unit2screen(np.array([ux, uy]))

    def units2rect(self, c1: np.ndarray, c2: np.ndarray) -> pygame.Rect:
        x1, y1 = self.unit2screen(c1)
        x2, y2 = self.unit2screen(c2)
        return pygame.Rect(x1, y1, x2 - x1, y2 - y1)


INIT_STATE_P = init.INIT_STATE_P
INIT_STATE_R = init.INIT_STATE_R
DEFAULT_KALMAN = init.DEFAULT_KALMAN


DEFAULT_PARAMETERS = init.DEFAULT_PARAMETERS

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = init.DEFAULT_REG

DEFAULT_REG_PID = init.DEFAULT_REG_PID


# Space = switch view mode (follow, free)
#   right-click drag = pan in free mode
# Tab = reset simulation
# Left/Right = control motor



class Render:
    def __init__(
        self,
        screen: pygame.surface.Surface,
        simulator_r: Simulator_Roll,
        simulator_p: Simulator_Pitch,
        init_state_r: SimulationState_Roll,
        init_state_p: SimulationState_Pitch,
        reg_p: Regulator,
        reg_r: Regulator,
        kalman_filter: KalmanFilter,
    ) -> None:
        self.screen = screen


        self.sim_p = simulator_p
        self.sim_r = simulator_r
        self.init_state_p = init_state_p
        self.init_state_r = init_state_r
        self.state_p = init_state_p
        self.state_r = init_state_r

        self.reg_p = reg_p
        self.reg_r = reg_r

        self.init_reg = reg
        self.filter_reg = reg
        self.current_signals = ControlSignals()
        #self.filter_sig = ControlSignals()

        self.init_kalman = kalman_filter
        self.filter = kalman_filter
        self.filter_state = init_state_p

        self.done = False
        self.space_p = ScreenSpaceTranslator(200, np.array([0., 0.,]), self.screen.get_size())
        self.space_r = ScreenSpaceTranslator(200, np.array([0., 0.,]), self.screen.get_size())
        self.wanted_zoom = self.space_p.pixels_per_unit

        self.mode = "follow" # "follow" follows the vehicle, "free_cam" allows for scrolling around with right-click
        self.speed_mult = 1.0

        self.font = pygame.font.Font(*INFO_FONT)
        self.info_height = 300

        # used in self.draw, size is set when needed, according to self.screen_split_at
        self.screen_width = self.screen.get_width()
        self.screen_height = self.screen.get_height()
        self.surf_pitch = pygame.surface.Surface((1,1))
        self.surf_roll = pygame.surface.Surface((1,1))
        self.surf_info_pitch = pygame.surface.Surface((1,1))
        self.surf_info_roll = pygame.surface.Surface((1,1))

        self.current_fps = 0.
        self.avg_tick_time = 0.

        self.sensor_reading = np.array([0.0,0.0,0.0,0.0])

        ### Initialize pointer objects for the c kalman filter
        self.c_state = pointer(States(init_state_p.top_angle,
                                      init_state_p.top_angle_d, 
                                      init_state_p.wheel_position,
                                      init_state_p.wheel_position_d, 
                                      init_state_r.top_angle, 
                                      init_state_r.top_angle_d))
        
        self.q_pitch = pointer(Matrix(init.Q_t[0][0] ,init.Q_t[0][1], init.Q_t[1][0], init.Q_t[1][1]))
        self.q_roll = pointer(Matrix(init.Q_r[0][0] ,init.Q_r[0][1], init.Q_r[1][0], init.Q_r[1][1]))
        self.q_wheel = pointer(Matrix(init.Q_w[0][0] ,init.Q_w[0][1], init.Q_w[1][0], init.Q_w[1][1]))

        cov_pitch = Matrix(init.P_t[0][0],init.P_t[0][1],init.P_t[1][0],init.P_t[1][1])
        cov_roll = Matrix(init.P_t[0][0],init.P_t[0][1],init.P_t[1][0],init.P_t[1][1])
        cov_wheel = Matrix(init.P_w[0][0],init.P_w[0][1],init.P_w[1][0],init.P_w[1][1])

        self.covariances = pointer(Covariances(cov_pitch, cov_roll, cov_wheel))

        self.r_vals = pointer(R_error(0.3, 0.3, 20))
        dt = init.dt

        self.reg_version = "Python" #C
        self.filter_version = "Python" #C

        self.a = 0
        
        #Variance for noise
        self.var_x = 0.0 #000004 #m/s^2
        self.var_z = 0.0 #000004 #m/s^2
        self.var_angle = 0.3 #rad/s
        self.var_wheel = 20 #0.00000001 #rpm

    def run(self) -> None:
        last_t = time.time()
        frames_last_second = []
        tick_times = []

        while not self.done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.done = True

                if event.type == pygame.MOUSEMOTION:
                    left, mid, right = pygame.mouse.get_pressed(3) # type: ignore
                    if right:
                        relx, rely = event.rel
                        self.space.view_center -= np.array([relx, -rely]) / self.space.pixels_per_unit

                if event.type == pygame.MOUSEWHEEL:
                    self.wanted_zoom *= 1.01 ** event.y

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.mode == "free_cam":
                            self.mode = "follow"
                        elif self.mode == "follow":
                            self.mode = "free_cam"
                    if event.key == pygame.K_TAB:
                        self.state_p = self.init_state_p
                        self.state_r = self.init_state_r
                        self.filter_state = self.init_state_p
                        self.filter = self.init_kalman
                        if isinstance(self.reg_p, PIDController):
                            self.reg_p.setpoint = 0
                        if isinstance(self.reg_p, LookaheadSpeedRegulator):
                            self.reg_p.setpoint_x_d = 0

            self.draw()

            pygame.display.flip()
            frames_last_second.append(time.time())

            if isinstance(self.sim_p, Resimulator_Pitch):
                dt = self.sim_p.get_dt()
                if dt < 0:
                    dt = 0.01
                time.sleep(dt)
                self.step(dt)

                frames_last_second = [t for t in frames_last_second if t > time.time() - 1]
                self.current_fps = len(frames_last_second)
                tick_times.append(dt)

            else: 
                dt = time.time() - last_t

                while dt > MIN_DT:
                    tick_start = time.time()
                    self.step(MIN_DT)
                    self.step_reg_filter_pitch(MIN_DT)
                    self.step_reg_filter_roll(MIN_DT)
                    tick_times.append(time.time() - tick_start)

                    dt -= MIN_DT
                self.step(dt)
                self.step_reg_filter_pitch(MIN_DT)
                self.step_reg_filter_roll(MIN_DT)

                last_t = time.time()

                frames_last_second = [t for t in frames_last_second if t > time.time() - 1]
                self.current_fps = len(frames_last_second)
            tick_times = tick_times[-1000:]
            self.avg_tick_time = sum(tick_times) / len(tick_times)

            


    def step_reg_filter_pitch(self, dt: float) -> None:

        mult = 3. if pygame.key.get_pressed()[pygame.K_LALT] else 0.3 if pygame.key.get_pressed()[pygame.K_LSHIFT] else 1.0
        val = mult if pygame.key.get_pressed()[pygame.K_RIGHT] else -mult if pygame.key.get_pressed()[pygame.K_LEFT] else 0

        if pygame.key.get_pressed()[pygame.K_UP]:
            if isinstance(self.reg_p, LookaheadSpeedRegulator):
                self.reg_p.setpoint_x_d += 0.001
            if isinstance(self.reg_p, PIDController):
                self.reg_p.setpoint += 0.0001
        if pygame.key.get_pressed()[pygame.K_DOWN]:
            if isinstance(self.reg_p, LookaheadSpeedRegulator):
                self.reg_p.setpoint_x_d -= 0.001
            if isinstance(self.reg_p, PIDController): 
                self.reg_p.setpoint -= 0.0001
        if pygame.key.get_pressed()[pygame.K_1]:
            if isinstance(self.reg_r, PIDController):
                self.reg_r.setpoint += 0.0001
        if pygame.key.get_pressed()[pygame.K_2]:
            if isinstance(self.reg_r, PIDController):
                self.reg_r.setpoint -= 0.0001
    
        elif pygame.key.get_pressed()[pygame.K_s]:
            self.speed_mult *= 2. ** (val * dt)
        else:
            self.current_signals.motor_torque_signal_pitch += val * 30
            self.current_signals.motor_torque_signal_roll += val * 30
            #self.filter_sig.motor_torque_signal += val * 30

        sim_dt = dt * self.speed_mult
    
        if self.filter_version == "C":
            ######## Kalman filter in C ##########
            #Update the Qs since they depend on dt
            self.q_pitch.contents.m11 = c_float(init.q_tc * (dt**4)/4) 
            self.q_pitch.contents.m12 = c_float(init.q_tc * (dt**3)/2)
            self.q_pitch.contents.m21 = c_float(init.q_tc * (dt**3)/2)
            self.q_pitch.contents.m22 = c_float(init.q_tc * dt**2)

            self.q_wheel.contents.m11 = c_float(init.q_wc * (dt**4)/4) 
            self.q_wheel.contents.m12 = c_float(init.q_wc * (dt**3)/2)
            self.q_wheel.contents.m21 = c_float(init.q_wc * (dt**3)/2)
            self.q_wheel.contents.m22 = c_float(init.q_wc * dt**2)

            c_kalman.kalman_filter_predict(c_float(0.0), c_float(dt), self.c_state, self.q_pitch, self.q_wheel, self.covariances)

            self.filter_state.top_angle = self.c_state.contents.x1
            self.filter_state.top_angle_d = self.c_state.contents.x2
            self.filter_state.wheel_position = self.c_state.contents.x3 
            self.filter_state.wheel_position_d = self.c_state.contents.x4

            ######################################
    
        if self.filter_version == "Python":
            ####### Python kalman filter #########
            kalman_out = self.filter.predict(self.a)
            self.filter_state.top_angle = kalman_out[0][0]  
            self.filter_state.top_angle_d = kalman_out[1][0]  
            self.filter_state.wheel_position = kalman_out[2][0]  
            self.filter_state.wheel_position_d = kalman_out[3][0]  
            ######################################

        if self.reg_version == "Python":
            ############### Regulator in python ########
            self.current_signals.motor_torque_signal_pitch = self.reg_p(self.filter_state, dt * self.speed_mult)
            ############################################

        if self.reg_version == "C":
            ############## Regulator in c ##############
            ## Update params in regulator.h if changed in sim_p
            c_regulator = reg.LookaheadSpeedRegulator
            c_regulator.restype = c_float  # Set output type from c code 

            
            
            self.current_signals = ControlSignals(c_regulator(c_float(self.reg_p.setpoint_x_d), 
                                                    c_float(self.filter_state.top_angle), 
                                                    c_float(self.filter_state.top_angle_d),
                                                    c_float(self.filter_state.wheel_position_d),
                                                    c_float(dt)))
            #############################################


        ############ Copy original state to filter state ###############
        self.filter_state = deepcopy(self.state_p)

        ########### Sensor reading and noise #######
        self.sensor_reading = self.sim_p.sensor_reading(self.filter_state, self.current_signals, (self.var_angle, self.var_wheel, self.var_x, self.var_z))
        
    
        
    

        if self.filter_version == "Python":
            ###### Python kalman filter #########
            #Update arrays that depend on dt
            F = np.array([[1, dt],
                               [0, 1]])
            Q =  np.array([[init.q_tc * (dt**4)/4, init.q_tc * (dt**3)/2, 0, 0],
                     [init.q_tc * (dt**3)/2,  init.q_tc * dt**2, 0, 0],
                     [0, 0, init.q_wc * (dt**4)/4, init.q_wc * (dt**3)/2],
                     [0, 0, init.q_wc * (dt**3)/2, init.q_wc * dt**2]])
            
            R = self.sim_p.params.sensor_position
            G = np.array([(0.5*dt**2)*R,dt*R]).reshape(2,1)

            self.filter.update(self.sensor_reading[0:1][0], F = F, Q = Q, G = G)
            print(self.filter.P)
            
            ######################################

        if self.filter_version == "C":
            ##### C Kalman filter #####
            c_kalman.kalman_filter_update(c_float(self.sensor_reading[0]), c_float(self.sensor_reading[1]), c_float(dt), self.c_state, self.q_pitch, self.q_wheel, self.covariances, self.r_vals)

            ###########################



    def step_reg_filter_roll(self, dt: float):

        #Call PID regulator and set torque signal for roll
        self.current_signals.motor_torque_signal_roll = self.reg_r(self.state_r, dt)


    def step(self, dt: float) -> None:
        
        sim_dt = dt
        ############ Integration step ###############
        self.state_p = self.sim_p.step(self.state_p, self.current_signals, sim_dt)
        self.state_r = self.sim_r.step(self.state_r, self.state_p, self.current_signals, sim_dt)

        #self.filter_state = self.sim_p.step(self.filter_state, self.current_signals, sim_dt)        


        self.space_p.pixels_per_unit = self.space_p.pixels_per_unit + (self.wanted_zoom - self.space_p.pixels_per_unit) * dt / ZOOM_TAU
        if self.mode == "follow":
            pos = np.array([self.state_p.wheel_position, self.sim_p.params.pitch_wheel_rad + 0.5])

            expected = pos + CAMERA_TAU * 0.8 * np.array([self.state_p.wheel_position_d, 0])
            self.space_p.view_center = self.space_p.view_center + (expected - self.space_p.view_center) * dt / CAMERA_TAU

    def draw(self) -> None:

        render_size = (self.screen.get_width()/2, self.screen.get_height() - self.info_height - BORDER)
        info_size = (self.screen.get_width()/2, self.info_height)

        if self.surf_pitch.get_size() != render_size:
            self.surf_pitch = pygame.surface.Surface(render_size)

        if self.surf_roll.get_size() != render_size:
            self.surf_roll = pygame.surface.Surface(render_size)

        if self.surf_info_pitch.get_size() != info_size:
            self.surf_info_pitch = pygame.surface.Surface(info_size)

        if self.surf_info_roll.get_size() != info_size:
            self.surf_info_roll = pygame.surface.Surface(info_size)

   
        self.space_p.set_screen_size(self.surf_pitch.get_size())
        self.space_r.set_screen_size(self.surf_roll.get_size())

        self.surf_pitch.fill((0, 0, 0))
        self.surf_roll.fill((0, 0, 0))

        self.draw_grid(self.surf_pitch, self.space_p)
        self.draw_grid(self.surf_roll, self.space_r)

        self.render_sim_pitch(self.surf_pitch, SimRenderOptions())
        self.render_sim_roll(self.surf_roll, SimRenderOptions_2())

        self.draw_info(self.surf_info_pitch, self.state_p)
        self.draw_info(self.surf_info_roll, self.state_r)

        self.screen.fill((255, 0, 255))
        self.screen.blit(self.surf_pitch, (0, 0))
        self.screen.blit(self.surf_roll, (self.screen_width/2, 0))
        self.screen.blit(self.surf_info_pitch, (0, self.screen.get_height() - info_size[1]))
        self.screen.blit(self.surf_info_roll, (self.screen_width/2, self.screen.get_height() - info_size[1]))

    def render_sim_pitch(self, surf: pygame.surface.Surface, render_options: SimRenderOptions, state_p: Optional[SimulationState_Pitch] = None, sim_p: Optional[Simulator_Pitch] = None) -> None:
        if state_p is None:
            state_p = self.state_p

        if sim_p is None:
            sim_p = self.sim_p

        wheel_center = np.array([state_p.wheel_position, sim_p.params.pitch_wheel_rad])
        wx, wy = self.space_p.unit2screen(wheel_center)
        if wx < -surf.get_width() or wx > surf.get_width() * 2 or wy < -surf.get_height() or wy > surf.get_height() * 2:
            return

        # spokes
        for spoke in range(render_options.n_spokes):
            inherit_angle = 2 * np.pi * spoke / render_options.n_spokes
            angle = inherit_angle - state_p.wheel_position / sim_p.params.pitch_wheel_rad

            r_out = sim_p.params.pitch_wheel_rad * (1 - render_options.outside_thickness * 0.5)
            r_in = sim_p.params.pitch_wheel_rad * render_options.inner_size * (1 - render_options.inner_thickness * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                surf,
                render_options.spoke_color,
                self.space_p.unit2screen(wheel_center + r_out * rot),
                self.space_p.unit2screen(wheel_center + r_in * rot),
                int(sim_p.params.pitch_wheel_rad * render_options.spoke_size * self.space_p.pixels_per_unit + 0.5),
            )

        # outer
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_p.unit2screen(wheel_center),
            sim_p.params.pitch_wheel_rad * self.space_p.pixels_per_unit,
            int(sim_p.params.pitch_wheel_rad * render_options.outside_thickness * self.space_p.pixels_per_unit + 0.5),
        )

        # inner
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_p.unit2screen(wheel_center),
            sim_p.params.pitch_wheel_rad * render_options.inner_size * self.space_p.pixels_per_unit,
            int(sim_p.params.pitch_wheel_rad * render_options.inner_size * render_options.inner_thickness * self.space_p.pixels_per_unit + 0.5),
        )

        if render_options.draw_torque:
            torque_rect_top_left = wheel_center - sim_p.params.pitch_wheel_rad * render_options.torque_size * np.array([0.5, -0.5])
            torque_rect_bottom_right = wheel_center + sim_p.params.pitch_wheel_rad * render_options.torque_size * np.array([0.5, -0.5])

            angle = -state_p.motor_torque / 10

            start, end = (np.pi / 2, np.pi / 2 + angle) if angle > 0 else (np.pi/2 + angle, np.pi / 2)

            pygame.draw.arc(
                surf,
                render_options.torque_color,
                self.space_p.units2rect(torque_rect_top_left, torque_rect_bottom_right),
                start - state_p.top_angle, end - state_p.top_angle,
                int(sim_p.params.pitch_wheel_rad * render_options.torque_size * 0.3 * self.space_p.pixels_per_unit), # this is incredibly ugly for some reason
            )

        # draw top
        top_angle = state_p.top_angle
        top_center = wheel_center + sim_p.params.chassi_mc_height * np.array([np.sin(top_angle), np.cos(top_angle)])
        
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_p.unit2screen(top_center),
            0.1 * self.space_p.pixels_per_unit,
        )

        pygame.draw.line(
            surf,
            render_options.spoke_color,
            self.space_p.unit2screen(top_center),
            self.space_p.unit2screen(wheel_center),
            3,
        )

        if render_options.draw_sensor:
            sensor_center = wheel_center + sim_p.params.sensor_position * np.array([np.sin(top_angle), np.cos(top_angle)])

            pygame.draw.circle(
                surf,
                render_options.sensor_color,
                self.space_p.unit2screen(sensor_center),
                0.03 * self.space_p.pixels_per_unit,
            )

            x_hat, z_hat = np.array(sim_p.sensor_axes(state_p)) # type: ignore
            a_x = self.sensor_reading[2]
            a_z = self.sensor_reading[3]
            x_vec, z_vec = a_x * x_hat, a_z * z_hat

            pygame.draw.line(
                surf,
                render_options.sensor_measure_color,
                self.space_p.unit2screen(sensor_center),
                self.space_p.unit2screen(sensor_center + x_vec),
                3
            )

            pygame.draw.line(
                surf,
                render_options.sensor_measure_color,
                self.space_p.unit2screen(sensor_center),
                self.space_p.unit2screen(sensor_center + z_vec),
                3
            )


    def render_sim_roll(self, surf: pygame.surface.Surface, render_options: SimRenderOptions) -> None:
        self.space_r.set_screen_size(surf.get_size())

        self.pos = self.sim_r.get_pos_vel(self.state_r)

        wheel_center = np.array([self.pos.top_center_pos_x, self.pos.top_center_pos_z ])
        wx, wy = self.space_r.unit2screen(wheel_center)
        if wx < -surf.get_width() or wx > surf.get_width() * 2 or wy < -surf.get_height() or wy > surf.get_height() * 2:
            return

        # spokes
        for spoke in range(render_options.n_spokes):
            inherit_angle = 2 * np.pi * spoke / render_options.n_spokes
            angle = inherit_angle - (self.state_r.reaction_wheel_angle) / self.sim_r.params.roll_wheel_rad

            r_out = self.sim_r.params.roll_wheel_rad * (1 - render_options.inner_thickness * 0.5)
            r_in = self.sim_r.params.roll_wheel_rad * render_options.inner_size * (1 - render_options.inner_thickness * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                surf,
                render_options.spoke_color,
                self.space_r.unit2screen(wheel_center + r_out * rot),
                self.space_r.unit2screen(wheel_center + r_in * rot),
                int(self.sim_r.params.roll_wheel_rad * render_options.spoke_size * self.space_r.pixels_per_unit + 0.5),
            )

        # outer
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_r.unit2screen(wheel_center),
            self.sim_r.params.roll_wheel_rad * self.space_r.pixels_per_unit,
            int(self.sim_r.params.roll_wheel_rad * render_options.outside_thickness * self.space_r.pixels_per_unit + 0.5),
        )

        # inner
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_r.unit2screen(wheel_center),
            self.sim_r.params.roll_wheel_rad * render_options.inner_size * self.space_r.pixels_per_unit,
            int(self.sim_r.params.roll_wheel_rad * render_options.inner_size * render_options.inner_thickness * self.space_r.pixels_per_unit + 0.5),
        )

        # torque
        torque_rect_top_left = wheel_center - self.sim_r.params.roll_wheel_rad * render_options.torque_size * np.array([0.5, -0.5])
        torque_rect_bottom_right = wheel_center + self.sim_r.params.roll_wheel_rad * render_options.torque_size * np.array([0.5, -0.5])

        angle = -self.state_r.motor_torque / 10

        start, end = (np.pi / 2, np.pi / 2 + angle) if angle > 0 else (np.pi/2 + angle, np.pi / 2)

        pygame.draw.arc(
            surf,
            render_options.torque_color,
            self.space_r.units2rect(torque_rect_top_left, torque_rect_bottom_right),
            start - self.state_r.top_angle, end - self.state_r.top_angle,
            int(self.sim_r.params.roll_wheel_rad * render_options.torque_size * 0.3 * self.space_r.pixels_per_unit), # this is incredibly ugly for some reason
        )

        # draw top
        top_angle = self.state_r.top_angle
        base_center = np.array([0,0])
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space_r.unit2screen(base_center),
            0.01 * self.space_r.pixels_per_unit,
        )

        pygame.draw.line(
            surf,
            render_options.spoke_color,
            self.space_r.unit2screen(base_center),
            self.space_r.unit2screen(wheel_center),
            3,
        )


    def draw_grid(self, surf: pygame.surface.Surface, space) -> None:
        (x0, y0), (x1, y1) = space.screen2unit((0, 0)), space.screen2unit(surf.get_size())
        if (math.isnan(x0) or math.isnan(x1)):
            x0 = 0.0
            x1 = 0.0
        for x in range(int(x0), int(x1 + 1)):
            pygame.draw.line(
                surf,
                LINE_COL if x != 0 else LINE_COL0,
                space.unit2screen_(x, y0),
                space.unit2screen_(x, y1),
                2 if x != 0 else 3
            )

        for y in range(int(y1), int(y0 + 1)):
            pygame.draw.line(
                surf,
                LINE_COL if y != 0 else LINE_COL0,
                space.unit2screen_(x0, y),
                space.unit2screen_(x1, y),
                2 if y != 0 else 3
            )

    def draw_info(self, surf: pygame.surface.Surface, state: SimulationState) -> None:
        #Clear surface of old rendering
        surf.fill((20, 20, 20))

        if isinstance(state, SimulationState_Pitch):

            left_col = [
                ("Position", state.wheel_position, "m"),
                ("Speed", state.wheel_position_d * 3600, "m/h"),
                ("Speed kalman", self.filter_state.wheel_position_d * 3600, "m/h"),
                ("Angle", state.top_angle / np.pi * 180, "deg"),
                ("Sensor reading", self.sensor_reading[0] / np.pi * 180, "deg/s"),
                ("Motor torque", state.motor_torque, "Nm"),
            ]
            if isinstance(self.reg_p, LookaheadSpeedRegulator):
                left_col.extend([
                    ("Speed setpoint", self.reg_p.setpoint_x_d * 3600, "m/h"),
                ])
            if isinstance(self.reg_p, PIDController):
                left_col.extend([
                    ("Speed setpoint", self.reg_p.setpoint / np.pi *180, "deg"),
                ])
    


        if isinstance(state, SimulationState_Roll):
            left_col = [
                ("Angle", state.top_angle / np.pi * 180, "deg"),
                ("Angle_d", state.top_angle_d / np.pi * 180, "deg/s"),
                ("R wheel angle", state.reaction_wheel_angle / np.pi * 180, "deg"),
                ("R wheel angle_d", state.reaction_wheel_angle_d / np.pi * 180, "deg"),
                ("Motor torque", state.motor_torque, "Nm"),

            ]
            if isinstance(self.reg_r, PIDController):
                left_col.extend([
                    ("Speed setpoint", self.reg_r.setpoint / np.pi *180, "deg"),
                ])

        #Rendering of left col
        y = 10
        for name, val, unit in left_col:
            text = f"{name}: {fmt_unit(val, unit)}"

            rendered = self.font.render(text, True, (255, 255, 255))
            surf.blit(rendered, (10, y))
            y += self.font.get_linesize()

        #Rendering of right col
        y = 10  
        for name, val, unit in [
            ("Speed", self.speed_mult, "s/s"),
            ("Frame rate", self.current_fps, "FPS"),
            ("Tick time", self.avg_tick_time, "s"),
        ]:
            text = f"{fmt_unit(val, unit)}: {name}"

            rendered = self.font.render(text, True, (255, 255, 255))
            surf.blit(rendered, (surf.get_width() - 10 - rendered.get_width(), y))
            y += self.font.get_linesize()


        


pygame.init()
pygame.font.init()  
screen = pygame.display.set_mode((1900, 1000), pygame.RESIZABLE, vsync=1)
pygame.display.set_caption("Autonomous Unicycle")




r_resim = Render(
    screen,
    Simulator_Roll(DEFAULT_PARAMETERS),
    Resimulator_Pitch("/home/andreas/School-local/Kandidatarbete/autonomous_unicycle/sensor_data/run-4.txt", DEFAULT_PARAMETERS),
    INIT_STATE_R,
    INIT_STATE_P,
    reg_p = DEFAULT_REG,
    reg_r= DEFAULT_REG_PID,
    kalman_filter= DEFAULT_KALMAN
)


r = Render(
    screen,
    Simulator_Roll(DEFAULT_PARAMETERS),
    Simulator_Pitch(DEFAULT_PARAMETERS),
    INIT_STATE_R,
    INIT_STATE_P,
    reg_p = DEFAULT_REG,
    reg_r= DEFAULT_REG_PID,
    kalman_filter= DEFAULT_KALMAN
)

#r.run()

r_resim.run()