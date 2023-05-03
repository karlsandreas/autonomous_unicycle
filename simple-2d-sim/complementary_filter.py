
class Complementary_Filter():
    def __init__(self, 
                 weight_gyro : float,
                 weight_acclerometer : float,
                 init_angle : float):
        self.weight_gyro = weight_gyro
        self.weight_accelerometer = weight_acclerometer
        self.angle = init_angle

    def __call__(self, dt, gyro_reading, accelerometer_reading):
        
        self.angle = self.weight_gyro * (self.angle + gyro_reading * dt) + self.weight_accelerometer * accelerometer_reading

        return self.angle