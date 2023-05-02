from typing import Tuple
import numpy as np
from debugparser import DebugParser
from sim import SimulationState_Pitch, SimulationState_Roll, Simulator_Pitch, Simulator_Roll, ControlSignals



    
class Resimulator_Pitch(Simulator_Pitch):
    def __init__(self, path, params):
        self.params = params
        self.dbgparser = DebugParser(path=path)
        self.dbgparser.parse()
        self.i = 0
        self.df_length = self.dbgparser.df.shape[0]

    def sensor_reading(self, state: SimulationState_Pitch, signals: ControlSignals, variances) -> np.array([float, float, float, float]):
        accelerations = self.dbgparser.get_acceleration(self.i)
        current_readings = self.dbgparser.get_current_readings(self.i)

        return np.array([current_readings["gx"],current_readings["erpm"]/22.9,0,accelerations[0],accelerations[2]])

    def sensor_axes(self, state: SimulationState_Pitch) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        dummy_return = (0,0)
        return (dummy_return, dummy_return)
    
    def get_dt(self):
        if self.i <= self.df_length:
            return self.dbgparser.get_dt(self.i)
        else:
            return None

    def step(self, state: SimulationState_Pitch, signals: ControlSignals, dt: float) -> SimulationState_Pitch:
        if self.i <= self.df_length:
            self.i += 1
            return self.dbgparser.get_Simstate_Pitch(self.i)
        else:
            return None