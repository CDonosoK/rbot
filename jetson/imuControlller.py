import serial

class imuController:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1)
        self.ser.flush()
        self.protocol_header = 0x55
        self.protocol_time = 0x50
        self.protocol_accel = 0x51
        self.protocol_gyro = 0x52
        self.protocol_angle = 0x53
        self.protocol_magnet = 0x54
        self.protocol_pressure = 0x56
        self.protocol_gps = 0x57
        self.protocol_qtn = 0x59

        self.accel = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.angle = [0, 0, 0]
        self.magnet = [0, 0, 0]
        self.pressure = 0
        self.gps = [0, 0, 0]
        self.qtn = [0, 0, 0, 0]


    def read(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline()
            


if __name__ == '__main__':
    imu = imuController()
    while True:
        imu.read()
    