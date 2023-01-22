
class PIDcontroller:
    def __init__(
        self,
        kp,
        ki,
        kd,
        setpoint):  
        self.kp = kp
        self.ki = ki
        self.kd = kd 
        self.setpoint = setpoint   
        self.lasterror = 0.0 # Used to calculate derivate
        self.ierror = 0.0

    def __call__(self,signal,dt):

        error = self.setpoint - signal
        if self.lasterror == 0.0:
            derror = 0.0 #Avoid very big error first itteration
        else:
            derror = (error - self.lasterror) /(dt) 

        self.ierror += error * dt #Accumulate integration error 
        self.lasterror = error

        out = self.kp * error + self.ki * self.ierror + self.kd * derror

        return out
        