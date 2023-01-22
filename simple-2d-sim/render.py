from typing import Tuple

import pygame
import numpy as np
import random

from pidcontroller import PIDcontroller

import time

from sim import SimulationState, RungeKuttaIntegrator

# Parameters for rendering
LINE_COL = (64, 64, 64)
LINE_COL0 = (100, 100, 100)
WHEEL_COLOR = (200, 200, 200)
SPOKE_COLOR = (150, 150, 150)

TORQUE_COLOR = (255, 0, 0)
SETPOINT_COLOR = (0, 255, 0)

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
        screen_size: Tuple[int, int], # TODO: Should be updated by Render when window changes size
    ):
        self.pixels_per_unit = pixels_per_unit
        self.view_center = view_center
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

DEFAULT_SIM = SimulationState(
    np.array([
        0, # torque_wanted
        9.82, # g
        0.28, # r0
        0.8, # R1
        1, # m0
        1, # m1
        0.1, # motor_tau
    ]),
    np.array([
        0, # x
        0, # x_d
        np.pi / 2, # Θ1
        0, # Θ1_d
        0, # motor_torque
    ]),
)
DEFAULT_PID = PIDcontroller(14.0, #Kp
                            0.9, #Ki
                            6.0,#Kd
                            np.pi/2)   #setpoint, in this case, the angle

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
        pid: PIDcontroller,
    ) -> None:
        self.screen = screen

        self.sim = sim
        self.integrator = integrator
        self.pid = pid
        self.done = False


        self.space = ScreenSpaceTranslator(200, np.array([0., 0.,]), self.screen.get_size())

        self.mode = "follow" # "follow" follows the vehicle, "free_cam" allows for scrolling around with right-click

        self.speed_mult = 0.5

    def run(self) -> None:
        last_t = time.time()

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

            self.render()

            pygame.display.flip()

            dt = time.time() - last_t
            while dt > MIN_DT:
                self.step(MIN_DT)
                dt -= MIN_DT
            self.step(dt)

            last_t = time.time()

    def draw_grid(self) -> None:
        (x0, y0), (x1, y1) = self.space.screen2unit((0, 0)), self.space.screen2unit(self.screen.get_size())
        for x in range(int(x0), int(x1 + 1)):
            pygame.draw.line(
                self.screen,
                LINE_COL if x != 0 else LINE_COL0,
                self.space.unit2screen_(x, y0),
                self.space.unit2screen_(x, y1),
                1 if x != 0 else 2
            )

        for y in range(int(y1), int(y0 + 1)):
            pygame.draw.line(
                self.screen,
                LINE_COL if y != 0 else LINE_COL0,
                self.space.unit2screen_(x0, y),
                self.space.unit2screen_(x1, y),
                1 if y != 0 else 2
            )

    def render(self) -> None:
        torque_wanted, g, r0, R1, m0, m1, motor_tau = self.sim.params
        x, x_d, θ1, θ1_d, torque = self.sim.state

        self.screen.fill((0, 0, 0))
        self.draw_grid()

        wheel_center = np.array([x, r0])

        # spokes
        for spoke in range(N_SPOKES):
            inherit_angle = 2 * np.pi * spoke / N_SPOKES
            angle = inherit_angle - x / r0

            r_out = r0 * (1 - OUTSIDE_THICKNESS * 0.5)
            r_in = r0 * INNER_SIZE * (1 - INNER_THICKNESS * 0.5)

            rot = np.array([np.cos(angle), np.sin(angle)])

            pygame.draw.line(
                self.screen,
                SPOKE_COLOR,
                self.space.unit2screen(wheel_center + r_out * rot),
                self.space.unit2screen(wheel_center + r_in * rot),
                int(r0 * SPOKE_SIZE * self.space.pixels_per_unit + 0.5),
            )
        # outer
        pygame.draw.circle(
            self.screen,
            WHEEL_COLOR,
            self.space.unit2screen(wheel_center),
            r0 * self.space.pixels_per_unit,
            int(OUTSIDE_THICKNESS * r0 * self.space.pixels_per_unit + 0.5),
        )
        # inner
        pygame.draw.circle(
            self.screen,
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
            self.screen,
            TORQUE_COLOR,
            self.space.units2rect(top_left, bottom_right),
            start, end,
            int(r0 * TORQUE_SIZE * 0.3 * self.space.pixels_per_unit), # this is incredibly ugly for some reason
        )

        # draw weight
        box_center = wheel_center + R1 * np.array([np.cos(θ1), np.sin(θ1)])
        pygame.draw.circle(
            self.screen,
            WHEEL_COLOR,
            self.space.unit2screen(box_center),
            0.1 * self.space.pixels_per_unit,
        )

        pygame.draw.line(
            self.screen,
            SPOKE_COLOR,
            self.space.unit2screen(box_center),
            self.space.unit2screen(wheel_center),
            3,
        )

        # draw setpoint

        setpoint_center = wheel_center + R1 * np.array([np.cos(self.pid.setpoint), np.sin(self.pid.setpoint)])
        pygame.draw.line(
            self.screen,
            SETPOINT_COLOR,
            self.space.unit2screen(wheel_center),
            self.space.unit2screen(setpoint_center),
            2,
        )

    def step(self, dt: float) -> None:
        noise = random.uniform(-1,1) * dt  #Simulation sensor noise
        angle_with_noise = self.sim.state[2] + noise

        pidout = -self.pid(angle_with_noise,dt)
        self.sim.params[0] = pidout

        if pygame.key.get_pressed()[pygame.K_LEFT]:
            self.pid.setpoint += dt * 0.1
        elif pygame.key.get_pressed()[pygame.K_RIGHT]:
            self.pid.setpoint -= dt * 0.1
        self.sim = self.integrator.step(self.sim, dt * self.speed_mult).limit()

        if self.mode == "follow":
            torque_wanted, g, r0, R1, m0, m1, motor_tau = self.sim.params
            x, x_d, θ1, θ1_d, torque = self.sim.state

            pos = np.array([x, r0])
            expected = pos + CAMERA_TAU * np.array([x_d, 0])

screen = pygame.display.set_mode((1000, 800))

r = Render(screen, DEFAULT_SIM, RungeKuttaIntegrator(), DEFAULT_PID)


r.run()
