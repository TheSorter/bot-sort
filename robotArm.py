import serial
import time


class RobotArm:

    def __init__(self, input_port='/dev/cu.usbmodemFA131'):
        self.port = input_port
        self.ser = serial.Serial(self.port, 9600)

    def hit_left(self):
        self.ser.write('1'.encode('utf-8'))
        time.sleep(2)

    def hit_right(self):
        self.ser.write('2'.encode('utf-8'))
        time.sleep(2)

    def hit_forward(self):
        self.ser.write('3'.encode('utf-8'))
        time.sleep(2)
