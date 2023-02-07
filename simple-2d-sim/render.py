from typing import Tuple

import pygame
import numpy as np
import random

from pidcontroller import PIDController

import time

from sim import SimulationState, SimulationParameters, ControlSignals, Simulator
from fmt import fmt_unit

# Parameters for rendering
BORDER = 4

LINE_COL = (64, 64, 64)
LINE_COL0 = (100, 100, 100)
WHEEL_COLOR = (200, 200, 200)
SPOKE_COLOR = (150, 150, 150)

TORQUE_COLOR = (255, 0, 0)
SETPOINT_COLOR = (0, 255, 0)
DRAW_EXPECTED = False
EXPECTED_COLOR = (0, 255, 255)

INFO_FONT = "ShareTech.ttf", 30 # path, size

# all relative to wheel diameter
OUTSIDE_THICKNESS = 0.2
INNER_SIZE = 0.6
INNER_THICKNESS = 0.2
N_SPOKES = 20
SPOKE_SIZE = 0.1

TORQUE_SIZE = 0.5

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

class Regulator:
    def __init__(
        self,
        setpoint,
    ):
        self.setpoint = setpoint
        self.lasterror = 0
    # Returns motor torque to apply
    def __call__(self, sim: SimulationState, params: SimulationParameters, dt) -> float:
        r = params.wheel_rad
        x = sim.wheel_position
        x_d = sim.wheel_position_d
        theta = sim.top_angle
        theta_d = sim.top_angle_d
        
        K2 = 32
        KD2 = 0.8
        
        error = self.setpoint - theta
        
        delta_e = (error - self.lasterror) / dt
        self.lasterror = error

        m =  K2 * error  + KD2 * (delta_e - theta_d)
        return m

DEFAULT_REGULATOR = Regulator(
    setpoint= 0
    )

INIT_STATE = SimulationState(
    wheel_position = 0,
    wheel_position_d = 0,
    top_angle = 0,
    top_angle_d = 0,
    motor_torque= 0
)

DEFAULT_PARAMETERS = SimulationParameters(
    wheel_rad = 0.28,
    wheel_mass = 5,
    top_height = 0.8,
    top_mass = 4,
    motor_reaction_speed = 0.1,
)


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
    ) -> None:
        self.screen = screen

        self.sim = simulator
        self.init_state = self.state = init_state
        self.reg = reg

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
        tau = self.reg(self.state, self.sim.params, dt)
        
        control_signal = ControlSignals(motor_torque_signal=tau)

        mult = 3. if pygame.key.get_pressed()[pygame.K_LALT] else 0.3 if pygame.key.get_pressed()[pygame.K_LSHIFT] else 1.0
        val = mult if pygame.key.get_pressed()[pygame.K_RIGHT] else -mult if pygame.key.get_pressed()[pygame.K_LEFT] else 0

        if pygame.key.get_pressed()[pygame.K_p]:
            self.reg.setpoint_x += val * dt * 3
        if pygame.key.get_pressed()[pygame.K_s]:
            self.speed_mult *= 2. ** (val * dt)
        else:
            self.reg.setpoint += val * 0.001

        self.state = self.sim.step(self.state, control_signal, dt * self.speed_mult)

        self.space.pixels_per_unit = self.space.pixels_per_unit + (self.wanted_zoom - self.space.pixels_per_unit) * dt / ZOOM_TAU
        if self.mode == "follow":
            pos = np.array([self.state.wheel_position, self.sim.params.wheel_rad])

            expected = pos + CAMERA_TAU * 0.8 * np.array([self.state.wheel_position_d, 0])
            self.space.view_center = self.space.view_center + (expected - self.space.view_center) * dt / CAMERA_TAU

    def draw(self) -> None:
        render_size = (self.screen.get_width(), self.screen.get_height() - self.info_height - BORDER)
        info_size = (self.screen.get_width(), self.info_height)

        if self.surf_render.get_size() != render_size:
            self.surf_render = pygame.surface.Surface(render_size)

        if self.surf_info.get_size() != info_size:
            self.surf_info = pygame.surface.Surface(info_size)

        self.render(self.surf_render)
        self.draw_info(self.surf_info)

        self.screen.fill((255, 0, 255))
        self.screen.blit(self.surf_render, (0, 0))
        self.screen.blit(self.surf_info, (0, self.screen.get_height() - info_size[1]))

    def render(self, surf: pygame.surface.Surface) -> None:
        self.space.set_screen_size(surf.get_size())

        surf.fill((0, 0, 0))
        self.draw_grid(surf)

        wheel_center = np.array([self.state.wheel_position, self.sim.params.wheel_rad])
        wx, wy = self.space.unit2screen(wheel_center)
        if wx < -surf.get_width() or wx > surf.get_width() * 2 or wy < -surf.get_height() or wy > surf.get_height() * 2:
            return

        # spokes
        for spoke in range(N_SPOKES):
            inherit_angle = 2 * np.pi * spoke / N_SPOKES
            angle = inherit_angle - self.state.wheel_position / self.sim.params.wheel_rad

            r_out = self.sim.params.wheel_rad * (1 - OUTSIDE_THICKNESS * 0.5)
            r_in = self.sim.params.wheel_rad * INNER_SIZE * (1 - INNER_THICKNESS * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                surf,
                SPOKE_COLOR,
                self.space.unit2screen(wheel_center + r_out * rot),
                self.space.unit2screen(wheel_center + r_in * rot),
                int(self.sim.params.wheel_rad * SPOKE_SIZE * self.space.pixels_per_unit + 0.5),
            )

        # outer
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(wheel_center),
            self.sim.params.wheel_rad * self.space.pixels_per_unit,
            int(self.sim.params.wheel_rad * OUTSIDE_THICKNESS * self.space.pixels_per_unit + 0.5),
        )

        # inner
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(wheel_center),
            self.sim.params.wheel_rad * INNER_SIZE * self.space.pixels_per_unit,
            int(self.sim.params.wheel_rad * INNER_SIZE * INNER_THICKNESS * self.space.pixels_per_unit + 0.5),
        )

        # torque
        torque_rect_top_left = wheel_center - self.sim.params.wheel_rad * TORQUE_SIZE * np.array([0.5, -0.5])
        torque_rect_bottom_right = wheel_center + self.sim.params.wheel_rad * TORQUE_SIZE * np.array([0.5, -0.5])

        angle = -self.state.motor_torque

        start, end = (np.pi / 2, np.pi / 2 + angle) if angle > 0 else (np.pi/2 + angle, np.pi / 2)

        pygame.draw.arc(
            surf,
            TORQUE_COLOR,
            self.space.units2rect(torque_rect_top_left, torque_rect_bottom_right),
            start - self.state.top_angle, end - self.state.top_angle,
            int(self.sim.params.wheel_rad * TORQUE_SIZE * 0.3 * self.space.pixels_per_unit), # this is incredibly ugly for some reason
        )

        # draw top
        top_angle = self.state.top_angle
        top_center = wheel_center + self.sim.params.top_height * np.array([np.sin(top_angle), np.cos(top_angle)])
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(top_center),
            0.1 * self.space.pixels_per_unit,
        )

        pygame.draw.line(
            surf,
            SPOKE_COLOR,
            self.space.unit2screen(top_center),
            self.space.unit2screen(wheel_center),
            3,
        )

        # Draw position setpoint
        # pos = self.reg.setpoint_x
        # position_setpoint_x, _ = self.space.unit2screen(np.array([pos, 0]))
        # pygame.draw.line(
        #     surf,
        #     SETPOINT_COLOR,
        #     (position_setpoint_x, 0),
        #     (position_setpoint_x, surf.get_height()),
        #     2,
        # )

        if DRAW_EXPECTED:
            pos_expected = self.reg.expected_x_after(self.state, self.reg.settle_time_x)
            position_expected_x, _ = self.space.unit2screen(np.array([pos_expected, 0]))
            pygame.draw.line(
                surf,
                EXPECTED_COLOR,
                (position_expected_x, 0),
                (position_expected_x, surf.get_height()),
                2,
            )

            angle_expected = self.reg.expected_theta_after(self.state, self.reg.settle_time_theta)
            angle_center = wheel_center + self.sim.params.top_height * np.array([np.sin(angle_expected), np.cos(angle_expected)])
            pygame.draw.line(
                surf,
                EXPECTED_COLOR,
                self.space.unit2screen(wheel_center),
                self.space.unit2screen(angle_center),
                2,
            )

    def draw_grid(self, surf: pygame.surface.Surface) -> None:
        (x0, y0), (x1, y1) = self.space.screen2unit((0, 0)), self.space.screen2unit(surf.get_size())
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

        y = 10
        for name, val, unit in [
            ("Position", self.state.wheel_position, "m"),
            ("Speed", self.state.wheel_position_d * 3600, "m/h"),
            ("Angle", self.state.top_angle / np.pi * 180, "deg"),
            ("Motor torque", self.state.motor_torque, "Nm"),
            ("Setpoint", self.reg.setpoint / np.pi * 180, "deg"),
    
            #("Distance to setpoint", self.state.wheel_position - self.reg.setpoint_x, "m"),
        ]:
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
screen = pygame.display.set_mode((1000, 600), pygame.RESIZABLE, vsync=1)
pygame.display.set_caption("Autonomous Unicycle")

r = Render(
    screen,
    Simulator(DEFAULT_PARAMETERS),
    INIT_STATE,
    DEFAULT_REGULATOR,
)

r.run()