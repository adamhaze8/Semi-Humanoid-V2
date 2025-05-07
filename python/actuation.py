import serial
import math
import numpy as np
from time import sleep
import os

arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

L1 = 1
L2 = 1


def drive(L, R):
    L = np.clip(L, -98, 98)
    R = np.clip(R, -98, 98)
    L = str(int(L / 2 + 50)).zfill(2)
    R = str(int(R / 2 + 50)).zfill(2)
    arduino.write(("D" + L + R + "\n").encode("utf-8"))


class joint:
    def __init__(self, name, pos):
        self.pos = pos
        self.name = name

    def rotate(self, angle):
        arduino.write(
            (str(self.name) + str(int(angle)) + "\n").encode("utf-8"))
        self.pos = int(angle)


##### Servos #####
LS = joint(name="LS", pos=-90)
LT = joint(name="LT", pos=0)
LE = joint(name="LE", pos=0)
LW = joint(name="LW", pos=0)
RS = joint(name="RS", pos=-90)
RT = joint(name="RT", pos=0)
RE = joint(name="RE", pos=0)
RW = joint(name="RW", pos=0)
NP = joint(name="NP", pos=0)
NT = joint(name="NT", pos=0)

##### Leaning Joints #####
LF = joint(name="LF", pos=0)
LL = joint(name="LL", pos=0)


def in_motion():
    while True:
        try:
            arduino.reset_input_buffer()
            arduino.write("inMotion\n".encode("utf-8"))
            while not arduino.in_waiting > 0:
                sleep(0.01)
            data = arduino.readline().decode("utf-8").rstrip()
            if data == "true":
                return True
            else:
                return False
        except serial.SerialException:
            sleep(0.1)
            continue


def speak(message):
    os.system('espeak "' + message + '"')


def clench_gripper():
    arduino.write(("clench" + "\n").encode("utf-8"))


def release_gripper():
    arduino.write(("release" + "\n").encode("utf-8"))
