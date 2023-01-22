from typing import Tuple

import pygame
import numpy as np
import random

from pidcontroller import PIDController

import time

from sim import SimulationState, RungeKuttaIntegrator
from fmt import fmt_unit

# Parameters for rendering
BORDER = 4

LINE_COL = (64, 64, 64)
LINE_COL0 = (100, 100, 100)
WHEEL_COLOR = (200, 200, 200)
SPOKE_COLOR = (150, 150, 150)

TORQUE_COLOR = (255, 0, 0)
SETPOINT_COLOR = (0, 255, 0)

INFO_FONT = "ShareTech.ttf", 30 # path, size

# all relative to wheel diameter
OUTSIDE_THICKNESS = 0.2
INNER_SIZE = 0.6
INNER_THICKNESS = 0.2
N_SPOKES = 20
SPOKE_SIZE = 0.1

TORQUE_SIZE = 0.5

CAMERA_TAU = 0.1

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
        angle_pid: PIDController,
    ):
        self.angle_pid = angle_pid

    # Returns motor torque to apply
    def __call__(self, sim: SimulationState, dt: float) -> float:
        torque_wanted, g, r0, R1, m0, m1, motor_tau = sim.params
        x, x_d, θ1, θ1_d, torque = sim.state

        # Calculate torque to counteract the static force of gravity on the top load
        F_g = m1 * g
        F_g1 = F_g * np.cos(θ1)
        torque_0 = -F_g1 / (R1 * m1)

        delta_torque = -self.angle_pid(θ1, dt)

        return torque_0 + delta_torque


DEFAULT_SIM = SimulationState(
    np.array([
        0, # torque_wanted
        9.82, # g
        0.28, # r0
        0.8, # R1
        1, # m0
        1, # m1
        0.001, # motor_tau
    ]),
    np.array([
        0, # x
        0, # x_d
        np.pi / 2, # Θ1
        0, # Θ1_d
        0, # motor_torque
    ]),
)

DEFAULT_REGULATOR = Regulator(
    angle_pid=PIDController(
        12.0, #Kp
        2.0, #Ki
        4.0, #Kd
        np.pi/2,
    )
)

# Space = switch view mode (follow, free)
#   right-click drag = pan in free mode
# Tab = reset simulation
# Left/Right = control motor
class Render:
    def __init__(
        self,
        screen: pygame.surface.Surface,

        sim: SimulationState,
        integrator: RungeKuttaIntegrator,
        regulator: Regulator,
    ) -> None:
        self.screen = screen

        self.sim = sim
        self.integrator = integrator
        self.regulator = regulator
        self.done = False

        self.space = ScreenSpaceTranslator(200, np.array([0., 0.,]), self.screen.get_size())

        self.mode = "follow" # "follow" follows the vehicle, "free_cam" allows for scrolling around with right-click

        self.speed_mult = 0.5

        self.font = pygame.font.Font(*INFO_FONT)

        self.info_height = int(self.screen.get_height() * 0.3)
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
                    self.space.pixels_per_unit *= 1.01 ** event.y

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.mode == "free_cam":
                            self.mode = "follow"
                        elif self.mode == "follow":
                            self.mode = "free_cam"
                    if event.key == pygame.K_TAB:
                        self.sim = DEFAULT_SIM

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
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            self.regulator.angle_pid.setpoint += dt * 0.3
        elif pygame.key.get_pressed()[pygame.K_RIGHT]:
            self.regulator.angle_pid.setpoint -= dt * 0.3

        noise = np.zeros_like(self.sim.state)
        # noise[2] = random.gauss(0,0.1) * dt ** 0.5 # Apply sensor noise to angle

        applied_torque = self.regulator(self.sim.apply_velocities(noise), dt)
        self.sim.params[0] = applied_torque

        self.sim = self.integrator.step(self.sim, dt * self.speed_mult).limit()

        if self.mode == "follow":
            torque_wanted, g, r0, R1, m0, m1, motor_tau = self.sim.params
            x, x_d, θ1, θ1_d, torque = self.sim.state

            pos = np.array([x, r0])
            expected = pos + CAMERA_TAU * 0.8 * np.array([x_d, 0])
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

        torque_wanted, g, r0, R1, m0, m1, motor_tau = self.sim.params
        x, x_d, θ1, θ1_d, torque = self.sim.state

        surf.fill((0, 0, 0))
        self.draw_grid(surf)

        wheel_center = np.array([x, r0])

        # spokes
        for spoke in range(N_SPOKES):
            inherit_angle = 2 * np.pi * spoke / N_SPOKES
            angle = inherit_angle - x / r0

            r_out = r0 * (1 - OUTSIDE_THICKNESS * 0.5)
            r_in = r0 * INNER_SIZE * (1 - INNER_THICKNESS * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                surf,
                SPOKE_COLOR,
                self.space.unit2screen(wheel_center + r_out * rot),
                self.space.unit2screen(wheel_center + r_in * rot),
                int(r0 * SPOKE_SIZE * self.space.pixels_per_unit + 0.5),
            )
        # outer
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(wheel_center),
            r0 * self.space.pixels_per_unit,
            int(OUTSIDE_THICKNESS * r0 * self.space.pixels_per_unit + 0.5),
        )
        # inner
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(wheel_center),
            r0 * INNER_SIZE * self.space.pixels_per_unit,
            int(INNER_SIZE * INNER_THICKNESS * r0 * self.space.pixels_per_unit + 0.5),
        )
        # torque
        top_left = wheel_center - r0 * TORQUE_SIZE * np.array([0.5, -0.5])
        bottom_right = wheel_center + r0 * TORQUE_SIZE * np.array([0.5, -0.5])

        angle = torque

        start, end = (np.pi / 2, np.pi / 2 + angle) if angle > 0 else (np.pi/2 + angle, np.pi / 2)

        pygame.draw.arc(
            surf,
            TORQUE_COLOR,
            self.space.units2rect(top_left, bottom_right),
            start, end,
            int(r0 * TORQUE_SIZE * 0.3 * self.space.pixels_per_unit), # this is incredibly ugly for some reason
        )

        # draw weight
        box_center = wheel_center + R1 * np.array([np.cos(θ1), np.sin(θ1)])
        pygame.draw.circle(
            surf,
            WHEEL_COLOR,
            self.space.unit2screen(box_center),
            0.1 * self.space.pixels_per_unit,
        )

        pygame.draw.line(
            surf,
            SPOKE_COLOR,
            self.space.unit2screen(box_center),
            self.space.unit2screen(wheel_center),
            3,
        )

        # draw angle setpoint

        angle_setpoint_center = wheel_center + R1 * np.array([np.cos(self.regulator.angle_pid.setpoint), np.sin(self.regulator.angle_pid.setpoint)])
        pygame.draw.line(
            surf,
            SETPOINT_COLOR,
            self.space.unit2screen(wheel_center),
            self.space.unit2screen(angle_setpoint_center),
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
                1 if x != 0 else 2
            )

        for y in range(int(y1), int(y0 + 1)):
            pygame.draw.line(
                surf,
                LINE_COL if y != 0 else LINE_COL0,
                self.space.unit2screen_(x0, y),
                self.space.unit2screen_(x1, y),
                1 if y != 0 else 2
            )

    def draw_info(self, surf: pygame.surface.Surface) -> None:
        torque_wanted, g, r0, R1, m0, m1, motor_tau = self.sim.params
        x, x_d, θ1, θ1_d, torque = self.sim.state

        surf.fill((20, 20, 20))

        y = 10
        for name, val, unit in [
            ("Frame rate", self.current_fps, "FPS"),
            ("Tick time", self.avg_tick_time, "s"),
            ("Position", x, "m"),
            ("Speed", x_d * 3600, "m/h"),
            ("Angle", 90 - θ1 / np.pi * 180, "deg"),
            ("Motor torque", torque, "Nm"),
        ]:
            text = f"{name}: {fmt_unit(val, unit)}"

            rendered = self.font.render(text, True, (255, 255, 255))
            surf.blit(rendered, (10, y))
            y += self.font.get_linesize()


pygame.init()
pygame.font.init()
screen = pygame.display.set_mode((1000, 800), pygame.RESIZABLE)

r = Render(screen, DEFAULT_SIM, RungeKuttaIntegrator(), DEFAULT_REGULATOR)


r.run()
