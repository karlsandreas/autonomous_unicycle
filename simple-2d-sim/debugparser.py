import pathlib
import numpy as np
import re
import pandas as pd

from sim import SimulationState_Pitch, SimulationState_Roll

class DebugParser:
    def __init__(self, path) -> None:
        self.path = pathlib.Path(path)
        self.data_dict = {}
        self.df = pd.DataFrame()
        
        self.last_t = 0 #For calculation of dt of log 

    def parse(self):
        with open(self.path.resolve()) as f:
            lines = f.readlines()
        #Find line with "Terminal ready", after that all debug data should come
        for i in range(len(lines)):
            if lines[i] == 'Terminal ready\n':
                debugdata = lines[i+1:]
        
        # use regular expression to extract key-value pairs from all lines
        data = []

        for line in debugdata:
            pairs = re.findall(r"(\w+)\s*=\s*([-.\w]+)", line)
            d = dict(pairs)
            d = {k: v.replace(",", "") for k, v in d.items()}
            d = {k: float(v) if re.match(r"^\-?\d+?\.\d+?$", v) else int(v) if v.isdigit() else v for k, v in d.items()}
            data.append(d)

        # convert list of dictionaries to pandas dataframe
        self.df = pd.DataFrame(data)
        #Remove Nans
        self.df = self.df.dropna(axis=0)
        #Convert all values from string to floats
        for col in self.df.columns: 
            self.df[col] = self.df[col].astype('float', errors='ignore')

    
        divide_by_1000 = ["t","I_w","ax","ay","az","theta","theta_d","x","x_d"]
        for col in divide_by_1000:
            self.df[col] = self.df[col]/1000


    def get_Simstate_Pitch(self, i):
        current = self.df.iloc[i]

        state_pitch = SimulationState_Pitch(top_angle=current["theta"], #[rad]
                                            top_angle_d=current["theta_d"], #[rad/s]
                                            wheel_position=current["x"], #[m]
                                            wheel_position_d=current["x_d"], #[m/s] 
                                            motor_torque=current["I_w"] * 0.59 # Convert from current to torque, 0.59 is the measured constant"
                                            ) 

        return(state_pitch)
    
    def get_dt(self, i):
        current = self.df.iloc[i]
        current_t = current["t"]
        
        if i == 0: #First index we used the average as dt
            dts = self.df["t"]
            dt = sum(dts)/len(dts)/1000 
        else:
            dt =  current_t - self.last_t
        
        self.last_t = current_t

        return dt
    

    def get_acceleration(self, i):
        current = self.df.iloc[i]
        acc_x = current["ax"]
        acc_y = current["ay"]
        acc_z = current["az"]
    
        return (acc_x, acc_y, acc_z)
    
    def get_current_readings(self,i):
        current = self.df.iloc[i]

        return current
    

    