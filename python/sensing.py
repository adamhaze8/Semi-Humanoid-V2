###################################### SETUP ######################################
import speech_recognition as sr
import RPi.GPIO as GPIO
import time
from time import sleep
import serial
from gps import *
import math
import cv2
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from i2c_hmc5883l import HMC5883
from mpu6050 import mpu6050
import smbus
import numpy as np
import termios
from actuation import *

lidar = serial.Serial("/dev/ttyUSB1", 115200, timeout=0)

interpreter = make_interpreter(
    "model/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite")
interpreter.allocate_tensors()
labels = read_label_file("model/coco_labels.txt")
inference_size = input_size(interpreter)

ir_sensor = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(ir_sensor, GPIO.IN)


def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        while True:
            audio = r.listen(source)
            try:
                return r.recognize_google(audio, language="en-US")
            except sr.UnknownValueError:
                speak("Speak up please")
            except sr.RequestError:
                speak("Somethings Wrong, I'd check connection")
            else:
                break


def get_longitude():
    gpsd = gps(mode=WATCH_ENABLE)
    nx = gpsd.next()
    if nx["class"] == "TPV":
        return getattr(nx, "lon", "Unknown")


def get_latitude():
    gpsd = gps(mode=WATCH_ENABLE)
    nx = gpsd.next()
    if nx["class"] == "TPV":
        return getattr(nx, "lat", "Unknown")


def get_heading():
    i2c_hmc5883l = HMC5883(gauss=1.3)
    i2c_hmc5883l.set_declination(14, 74)
    return i2c_hmc5883l.get_heading()[0]


def get_acceleration(axis):
    while True:
        try:
            sensor = mpu6050(0x68)
            acceleration = sensor.get_accel_data()
            if axis == "x":
                return acceleration["x"]
            elif axis == "y":
                return acceleration["y"]
            elif axis == "z":
                return acceleration["z"]
        except IOError:
            sleep(0.1)
            print("Accelerometer IO error")
            continue


def get_pitch():
    while True:
        try:
            sensor = mpu6050(0x68)
            acceleration = sensor.get_accel_data()
            x = acceleration["x"]
            y = acceleration["y"]
            z = acceleration["z"]
            pitch = 57.2958 * math.atan2(-x, z)
            return pitch
        except IOError:
            sleep(0.1)
            print("Accelerometer IO error")
            continue


def get_roll():
    while True:
        try:
            sensor = mpu6050(0x68)
            acceleration = sensor.get_accel_data()
            x = acceleration["x"]
            y = acceleration["y"]
            z = acceleration["z"]
            roll = 57.2958 * math.atan2(y, z)
            return roll
        except IOError:
            sleep(0.1)
            continue


def locate(item, cap):
    ret, frame = cap.read()
    cv2_im = frame
    cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
    cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
    run_inference(interpreter, cv2_im_rgb.tobytes())
    objs = get_objects(interpreter, 0.1)[:3]
    # cv2.imshow("frame", cv2_im)
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if labels.get(obj.id) == item:
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)
            percent = int(100 * obj.score)
            label = "{}% {}".format(percent, labels.get(obj.id, obj.id))
            # cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            # cv2_im = cv2.putText(
            # cv2_im, label, (x0, y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            object_location_x = x0 + (x1 - x0) / 2
            object_location_y = y0 + (y1 - y0) / 2
            object_width = x1 - x0
            object_height = y1 - y0
            return object_location_x, object_location_y, object_width, object_height


def get_distance():
    while True:
        try:
            lidar.reset_input_buffer()
            while not lidar.in_waiting > 8:
                sleep(0.005)
            data = lidar.read(9)  # read 9 bytes
            if data[0] == 0x59 and data[1] == 0x59:  # check first two bytes
                # distance in next two bytes
                distance = data[2] + data[3] * 256
                # signal strength in next two bytes
                strength = data[4] + data[5] * 256
                temperature = data[6] + data[7] * 256  # temp in next two bytes
                temperature = (temperature / 8.0) - \
                    256.0  # temp scaling and offset
                return distance / 100
        except termios.error:
            sleep(0.1)
            continue


def get_ir_state():
    sensor_state = GPIO.input(ir_sensor)
    return (sensor_state == GPIO.LOW)
