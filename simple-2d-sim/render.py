from typing import Tuple, Optional

import pygame
import numpy as np
import random

from pidcontroller import PIDController

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

import time

from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from regulator import Regulator, LookaheadSpeedRegulator, NullRegulator
from kalman import KalmanFilter

from fmt import fmt_unit

import initials as init

#Imports for c code
from ctypes import *
so_file = "C:/Users/ante_/Documents/Kandidatarbete/autonomous_unicycle/algo-c/regulator.so"
reg = CDLL(so_file)


# Parameters for rendering
BORDER = 4

LINE_COL = (64, 64, 64)
LINE_COL0 = (100, 100, 100)

Color = Tuple[int, int, int]


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

#INFO_FONT = "ShareTech.ttf", 30 # path, size
INFO_FONT = "./simple-2d-sim/ShareTech.ttf", 30

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



INIT_STATE = init.INIT_STATE
DEFAULT_KALMAN = init.DEFAULT_KALMAN


DEFAULT_PARAMETERS = init.DEFAULT_PARAMETERS

# DEFAULT_REG = NullRegulator(params=DEFAULT_PARAMETERS)
DEFAULT_REG = init.DEFAULT_REG

DEFAULT_KALMAN_GAIN = init.DEFAULT_KALMAN_GAIN

# Space = switch view mode (follow, free)
#   right-click drag = pan in free mode
# Tab = reset simulation
# Left/Right = control motor
class Render:
    def __init__(
        self,
        screen: pygame.surface.Surface,
        simulator: Simulator,
        init_state: SimulationState,
        reg: Regulator,
        kalman_filter: KalmanFilter,
    ) -> None:
        self.screen = screen


        self.sim = simulator
        self.init_state = self.state = init_state
        
        self.reg = reg
        self.filter_reg = reg
        self.current_signals = ControlSignals()
        self.filter_sig = ControlSignals()

        self.init_kalman = kalman_filter
        self.filter = kalman_filter
        self.filter_state = init_state

        self.done = False
        self.space = ScreenSpaceTranslator(200, np.array([0., 0.,]), self.screen.get_size())
        self.wanted_zoom = self.space.pixels_per_unit

        self.mode = "follow" # "follow" follows the vehicle, "free_cam" allows for scrolling around with right-click
        self.speed_mult = 1.0

        self.font = pygame.font.Font(*INFO_FONT)
        self.info_height = 300
        # used in self.draw, size is set when needed, according to self.screen_split_at
        self.surf_render = pygame.surface.Surface((1, 1))
        self.surf_info = pygame.surface.Surface((1, 1))

        self.current_fps = 0.
        self.avg_tick_time = 0.

        self.sensor_reading = 0.0

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
                        self.state = self.init_state
                        self.filter_state = self.init_state
                        self.filter = self.init_kalman

            self.draw()

            pygame.display.flip()
            frames_last_second.append(time.time())

            dt = time.time() - last_t
            while dt > MIN_DT:
                tick_start = time.time()
                self.step(MIN_DT)
                tick_times.append(time.time() - tick_start)

                dt -= MIN_DT
            self.step(dt)

            last_t = time.time()

            frames_last_second = [t for t in frames_last_second if t > time.time() - 1]
            self.current_fps = len(frames_last_second)
            tick_times = tick_times[-1000:]
            self.avg_tick_time = sum(tick_times) / len(tick_times)


    def step(self, dt: float) -> None:

        #self.filter_sig = self.filter_reg(self.filter_state, dt* self.speed_mult)
        #self.current_signals = self.reg(self.filter_state, dt * self.speed_mult)
        c_regulator = reg.LookaheadSpeedRegulator
        c_regulator.restype = c_float  # Set output type from c code 
        
        self.current_signals = ControlSignals(c_regulator(c_float(self.reg.setpoint_x_d), 
                                                c_float(self.filter_state.top_angle), 
                                                c_float(self.filter_state.top_angle_d),
                                                c_float(self.filter_state.wheel_position_d),
                                                c_float(dt)))

        mult = 3. if pygame.key.get_pressed()[pygame.K_LALT] else 0.3 if pygame.key.get_pressed()[pygame.K_LSHIFT] else 1.0
        val = mult if pygame.key.get_pressed()[pygame.K_RIGHT] else -mult if pygame.key.get_pressed()[pygame.K_LEFT] else 0

        if pygame.key.get_pressed()[pygame.K_p]:
            if isinstance(self.reg, LookaheadSpeedRegulator):
                self.reg.setpoint_x_d += val * dt * 3
        elif pygame.key.get_pressed()[pygame.K_s]:
            self.speed_mult *= 2. ** (val * dt)
        else:
            self.current_signals.motor_torque_signal += val * 30
            self.filter_sig.motor_torque_signal += val * 30

        sim_dt = dt * self.speed_mult

        sensor_reading = self.sim.sensor_reading(self.state, self.current_signals)
        
        var_x = 0.5 #m/s^2
        var_z = 0.5 #m/s^2
        var_angle = 0.005 #rad/s 
        noise_x, noise_z, noise_angle = random.gauss(0, var_x**0.5), random.gauss(0, var_z**0.5), random.gauss(0, var_angle**0.5)

        a_x = sensor_reading[1] + noise_x
        a_z = sensor_reading[2] + noise_z
        a = (a_x**2 + a_z**2)**0.5

        c_state = States(self.state.top_angle, self.state.top_angle_d, self.state.wheel_position, self.state.wheel_position_d)
        c_cov = Matrix()

        top_angle_d = sensor_reading[0] + noise_angle

        self.sensor_reading = top_angle_d   
        pitch_kalman_filter_predict_wrapper()
        kalman_out = self.filter.predict(a)

        self.filter_state.top_angle = kalman_out[0][0]  
        self.filter_state.top_angle_d = kalman_out[1][0]  

        self.state = self.sim.step(self.state, self.current_signals, sim_dt)
        #self.filter.step(sim_dt, self.current_signals)
        self.filter_state = self.sim.step(self.filter_state, self.current_signals, sim_dt)        

        F = np.array([[1, dt],
                           [0, 1]])
        Q = 0.05 * np.array([[dt**2, dt],
                           [dt, 1]])
        R = self.sim.params.sensor_position
        G = np.array([(0.5*dt**2)*R,dt*R]).reshape(2,1)

        self.filter.update(top_angle_d, F = F, Q = Q, G = G)

        self.space.pixels_per_unit = self.space.pixels_per_unit + (self.wanted_zoom - self.space.pixels_per_unit) * dt / ZOOM_TAU
        if self.mode == "follow":
            pos = np.array([self.state.wheel_position, self.sim.params.wheel_rad + 0.5])

            expected = pos + CAMERA_TAU * 0.8 * np.array([self.state.wheel_position_d, 0])
            self.space.view_center = self.space.view_center + (expected - self.space.view_center) * dt / CAMERA_TAU

    def draw(self) -> None:
        render_size = (self.screen.get_width(), self.screen.get_height() - self.info_height - BORDER)
        info_size = (self.screen.get_width(), self.info_height)

        if self.surf_render.get_size() != render_size:
            self.surf_render = pygame.surface.Surface(render_size)

        if self.surf_info.get_size() != info_size:
            self.surf_info = pygame.surface.Surface(info_size)

        self.space.set_screen_size(self.surf_render.get_size())

        self.surf_render.fill((0, 0, 0))
        self.draw_grid(self.surf_render)

        self.render_sim(self.surf_render, SimRenderOptions())
        self.render_sim(self.surf_render, SimRenderOptions(), self.filter_state)

        self.draw_info(self.surf_info)

        self.screen.fill((255, 0, 255))
        self.screen.blit(self.surf_render, (0, 0))
        self.screen.blit(self.surf_info, (0, self.screen.get_height() - info_size[1]))

    def render_sim(self, surf: pygame.surface.Surface, render_options: SimRenderOptions, state: Optional[SimulationState] = None, sim: Optional[Simulator] = None) -> None:
        if state is None:
            state = self.state

        if sim is None:
            sim = self.sim

        wheel_center = np.array([state.wheel_position, sim.params.wheel_rad])
        wx, wy = self.space.unit2screen(wheel_center)
        if wx < -surf.get_width() or wx > surf.get_width() * 2 or wy < -surf.get_height() or wy > surf.get_height() * 2:
            return

        # spokes
        for spoke in range(render_options.n_spokes):
            inherit_angle = 2 * np.pi * spoke / render_options.n_spokes
            angle = inherit_angle - state.wheel_position / sim.params.wheel_rad

            r_out = sim.params.wheel_rad * (1 - render_options.outside_thickness * 0.5)
            r_in = sim.params.wheel_rad * render_options.inner_size * (1 - render_options.inner_thickness * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                surf,
                render_options.spoke_color,
                self.space.unit2screen(wheel_center + r_out * rot),
                self.space.unit2screen(wheel_center + r_in * rot),
                int(sim.params.wheel_rad * render_options.spoke_size * self.space.pixels_per_unit + 0.5),
            )

        # outer
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space.unit2screen(wheel_center),
            sim.params.wheel_rad * self.space.pixels_per_unit,
            int(sim.params.wheel_rad * render_options.outside_thickness * self.space.pixels_per_unit + 0.5),
        )

        # inner
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space.unit2screen(wheel_center),
            sim.params.wheel_rad * render_options.inner_size * self.space.pixels_per_unit,
            int(sim.params.wheel_rad * render_options.inner_size * render_options.inner_thickness * self.space.pixels_per_unit + 0.5),
        )

        if render_options.draw_torque:
            torque_rect_top_left = wheel_center - sim.params.wheel_rad * render_options.torque_size * np.array([0.5, -0.5])
            torque_rect_bottom_right = wheel_center + sim.params.wheel_rad * render_options.torque_size * np.array([0.5, -0.5])

            angle = -state.motor_torque / 10

            start, end = (np.pi / 2, np.pi / 2 + angle) if angle > 0 else (np.pi/2 + angle, np.pi / 2)

            pygame.draw.arc(
                surf,
                render_options.torque_color,
                self.space.units2rect(torque_rect_top_left, torque_rect_bottom_right),
                start - state.top_angle, end - state.top_angle,
                int(sim.params.wheel_rad * render_options.torque_size * 0.3 * self.space.pixels_per_unit), # this is incredibly ugly for some reason
            )

        # draw top
        top_angle = state.top_angle
        top_center = wheel_center + sim.params.top_height * np.array([np.sin(top_angle), np.cos(top_angle)])
        pygame.draw.circle(
            surf,
            render_options.wheel_color,
            self.space.unit2screen(top_center),
            0.1 * self.space.pixels_per_unit,
        )

        pygame.draw.line(
            surf,
            render_options.spoke_color,
            self.space.unit2screen(top_center),
            self.space.unit2screen(wheel_center),
            3,
        )

        if render_options.draw_sensor:
            sensor_center = wheel_center + sim.params.sensor_position * np.array([np.sin(top_angle), np.cos(top_angle)])

            pygame.draw.circle(
                surf,
                render_options.sensor_color,
                self.space.unit2screen(sensor_center),
                0.03 * self.space.pixels_per_unit,
            )

            x_hat, z_hat = np.array(sim.sensor_axes(state)) # type: ignore
            sensor_reding = sim.sensor_reading(state, self.current_signals)
            a_x = sensor_reding[1]
            a_z = sensor_reding[2]

            x_vec, z_vec = a_x * x_hat, a_z * z_hat

            pygame.draw.line(
                surf,
                render_options.sensor_measure_color,
                self.space.unit2screen(sensor_center),
                self.space.unit2screen(sensor_center + x_vec),
                3
            )

            pygame.draw.line(
                surf,
                render_options.sensor_measure_color,
                self.space.unit2screen(sensor_center),
                self.space.unit2screen(sensor_center + z_vec),
                3
            )

    def draw_grid(self, surf: pygame.surface.Surface) -> None:
        (x0, y0), (x1, y1) = self.space.screen2unit((0, 0)), self.space.screen2unit(surf.get_size())
        if (math.isnan(x0) or math.isnan(x1)):
            x0 = 0.0
            x1 = 0.0
        for x in range(int(x0), int(x1 + 1)):
            pygame.draw.line(
                surf,
                LINE_COL if x != 0 else LINE_COL0,
                self.space.unit2screen_(x, y0),
                self.space.unit2screen_(x, y1),
                2 if x != 0 else 3
            )

        for y in range(int(y1), int(y0 + 1)):
            pygame.draw.line(
                surf,
                LINE_COL if y != 0 else LINE_COL0,
                self.space.unit2screen_(x0, y),
                self.space.unit2screen_(x1, y),
                2 if y != 0 else 3
            )

    def draw_info(self, surf: pygame.surface.Surface) -> None:
        surf.fill((20, 20, 20))

        left_col = [
            ("Position", self.state.wheel_position, "m"),
            ("Speed", self.state.wheel_position_d * 3600, "m/h"),
            ("Speed kalman", self.filter_state.wheel_position_d * 3600, "m/h"),
            ("Angle", self.state.top_angle / np.pi * 180, "deg"),
            ("Sensor reading", self.sensor_reading / np.pi * 180, "deg/s"),
            # ("Linearity", np.sin(self.state.top_angle) / self.state.top_angle * 100, "%"),
            ("Motor torque", self.state.motor_torque, "Nm"),

        ]
        if isinstance(self.reg, LookaheadSpeedRegulator):
            left_col.extend([
                ("Speed setpoint", self.reg.setpoint_x_d * 3600, "m/h"),
            ])

        y = 10
        for name, val, unit in left_col:
            text = f"{name}: {fmt_unit(val, unit)}"

            rendered = self.font.render(text, True, (255, 255, 255))
            surf.blit(rendered, (10, y))
            y += self.font.get_linesize()

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
screen = pygame.display.set_mode((1000, 1000), pygame.RESIZABLE, vsync=1)
pygame.display.set_caption("Autonomous Unicycle")

r = Render(
    screen,
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REG,
    DEFAULT_KALMAN
)

r.run()
