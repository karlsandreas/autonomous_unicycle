
class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.lasterror = 0.0 # Used to calculate derivate
        self.ierror = 0.0
        self.has_limits = False #Optional with limits

    def __call__(self, signal: float, dt: float) -> float:
        #Run limit check on the setpoint
        self.setpoint = self.check_limits(self.setpoint)
        error = self.setpoint - signal
        

        if self.lasterror == 0.0:
            derror = 0.0 # Avoid very big error first itteration
        else:
            derror = (error - self.lasterror) / dt

        self.ierror += error * dt # Accumulate integration error
        self.lasterror = error

        out = self.kp * error + self.ki * self.ierror + self.kd * derror    
        
        return out


    def init_limits(self, upper: float, lower: float):   
        self.has_limits = True
        self.upper = upper
        self.lower = lower

    def check_limits(self, signal: float) -> float:
        if self.has_limits:  
            if signal > self.upper:
                return self.upper
            elif signal < self.lower:
                return self.lower
            else:
                return signal
        else: 
            return signal



