###################################### SETUP ######################################
import speech_recognition as sr
import time
from time import sleep
import os
import pyaudio
import serial
import math
import smbus
import threading
import numpy as np
import termios
from chatbot import BorisChatbot
from sensing import *
from actuation import *
from arm_3_dof_planar import Arm_3_DOF_Planar
from pid_controller import PID_Controller
import cv2

chatbot = BorisChatbot()
arm = Arm_3_DOF_Planar(0.245, 0.203, 0.25)

drive_PID_controller = PID_Controller(0.5, 0, 0.1)
follow_PID_controller = PID_Controller(1, 0, 0.1)

arduino = serial.Serial("/dev/ttyACM0", 9600, timeout=1)


###################################### SUBROUTINES ######################################


def go_to(location):
    tar_longitude = location[0]
    tar_latitude = location[1]
    while (
        abs(tar_latitude - cur_latitude) > 0.000018
        or abs(tar_longitude - cur_longitude) > 0.000018
    ):
        cur_longitude = get_longitude()
        cur_latitude = get_latitude()
        angle = abs(
            math.degrees(
                math.atan(
                    (tar_latitude - cur_latitude) /
                    (tar_longitude - cur_longitude)
                )
            )
        )
        if (tar_latitude - cur_latitude) > 0 and (tar_longitude - cur_longitude) > 0:
            tar_heading = 90 - angle
        if (tar_latitude - cur_latitude) > 0 and (tar_longitude - cur_longitude) < 0:
            tar_heading = 270 + angle
        if (tar_latitude - cur_latitude) < 0 and (tar_longitude - cur_longitude) < 0:
            tar_heading = 270 - angle
        if (tar_latitude - cur_latitude) < 0 and (tar_longitude - cur_longitude) > 0:
            tar_heading = 90 + angle
        data = get_heading()
        cur_heading = data[0]
        while abs(tar_heading - cur_heading) > 10:
            if (tar_heading - cur_heading) > 0:
                drive("R")
            else:
                drive("L")
            data = get_heading()
            cur_heading = data[0]
            sleep(0.01)
        drive("F")
        sleep(0.01)
    drive("H")


def approach(item, desired_dist):
    eye = cv2.VideoCapture(2)
    frame_width = eye.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = eye.get(cv2.CAP_PROP_FRAME_HEIGHT)

    degrees_per_pixel = 90/frame_width

    previous_time = time.time()
    while True:
        errors = look_at(item, eye)
        if errors is None:
            drive(0, 0)
            print(str(item) + " not found")
            sleep(0.01)
        else:
            x_error, y_error = errors
            
            if (x_error < 30) and (x_error > -30):
                neck_tilt = getattr(NT, "pos")
                item_dist_from_sensor = get_distance()
                item_dist = item_dist_from_sensor * np.cos(np.deg2rad(neck_tilt))
                if item_dist < desired_dist:
                    item_dist_from_sensor = get_distance()  # Double check sensor
                    item_dist = item_dist_from_sensor * np.cos(np.deg2rad(neck_tilt))
                    if item_dist < desired_dist:
                        drive(0, 0)
                        break

            current_time = time.time()
            dt = current_time - previous_time
            previous_time = current_time
            x_error = getattr(NP, "pos") + (x_error * degrees_per_pixel)
            steering_rate = follow_PID_controller.update(x_error, dt)
            print(20 + steering_rate)
            #drive(20 - steering_rate, 20 + steering_rate)
            sleep(0.01)

    eye.release()


def calibrate_compass():
    i2c_hmc5883l = HMC5883(gauss=1.3)
    # Set declination according to your position
    i2c_hmc5883l.set_declination(14, 74)

    Xmin = 1000
    Xmax = -1000
    Ymin = 1000
    Ymax = -1000

    i = 0
    drive("L")
    while i < 1000:
        x, y, z = i2c_hmc5883l.get_axes()
        Xmin = min(x, Xmin)
        Xmax = max(x, Xmax)
        Ymin = min(y, Ymin)
        Ymax = max(y, Ymax)
        i = i + 1
        sleep(0.01)
    drive("H")

    xs = 1
    ys = (Xmax - Xmin) / (Ymax - Ymin)
    xb = xs * (1 / 2 * (Xmax - Xmin) - Xmax)
    yb = xs * (1 / 2 * (Ymax - Ymin) - Ymax)

    speak("listing calibration corrections")
    sleep(1)
    speak("xs=" + str(xs))
    sleep(1)
    speak("ys=" + str(ys))
    sleep(1)
    speak("xb=" + str(xb))
    sleep(1)
    speak("yb=" + str(yb))
    sleep(1)
    speak("please enter parameters into the i2c_hmc5883l python script")


def drive_forward_delay(delay):
    drive(80, 80)
    sleep(delay)
    drive(0, 0)


def drive_distance_PID(distance):
    previous_time = time.time()
    velocity = 0
    position = 0

    while position_error > 0:
        # Get accelerometer data
        acceleration_y = get_acceleration("y")

        # Measure time difference since the last iteration
        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time

        # Integrate acceleration to get velocity
        velocity += acceleration_y * dt

        # Integrate velocity to get position
        position += velocity * dt

        position_error = distance - position

        # Apply the PID
        output = drive_PID_controller.update(position_error, dt)

        drive(output, output)

        time.sleep(0.01)  # Adjust the sleep duration as needed

    drive(0, 0)


def name_location(location):
    globals()[location] = (get_longitude, get_latitude)


def look_at(item, eye):
    # This now needs to be continually executed
    frame_width = eye.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = eye.get(cv2.CAP_PROP_FRAME_HEIGHT)
    item_location = locate(item, eye)
    if item_location is not None:
        x, y, h, w = item_location
        x_error = x - (frame_width / 2)
        y_error = y - (frame_height / 2)
        NP.rotate(getattr(NP, "pos") + x_error * (10 / frame_width))
        NT.rotate(getattr(NT, "pos") - y_error * (10 / frame_height))
        while in_motion() == 1:
            sleep(0.01)
        return x_error, y_error
    else:
        return None


def grab(item):
    release_gripper()
    sleep(1)
    # This assumes object is already in frame of gripper camera
    hand = cv2.VideoCapture(0)
    frame_width = hand.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = hand.get(cv2.CAP_PROP_FRAME_HEIGHT)
    x_error = 100
    y_error = 100

    while True:
        current_stretch, current_elevation = arm.get_FK(
            getattr(LS, "pos"), getattr(LE, "pos"), getattr(LW, "pos")
        )
        item_location = locate(item, hand)
        if item_location:
            x, y, w, h = item_location
            x_error = x - (frame_width / 2)
            LT.rotate(getattr(LT, "pos") + x_error * (10 / frame_width))
            y_error = y - (frame_height / 2)
            stretch_jog = 0.5 * (1 / max(x_error, y_error))

            target_LS, target_LE, target_LW = arm.get_RK(
                current_stretch + stretch_jog, current_elevation - y_error / 10000
            )
            LS.rotate(target_LS)
            LE.rotate(target_LE)
            LW.rotate(target_LW)
            while in_motion():
                sleep(0.01)

        # if was last in the middle
        elif abs(x_error) < 50 and abs(y_error) < 50:
            target_LS, target_LE, target_LW = arm.get_RK(
                current_stretch + 0.005, current_elevation
            )
            LS.rotate(target_LS)
            LE.rotate(target_LE)
            LW.rotate(target_LW)
            while in_motion():
                sleep(0.01)
        if get_ir_state():
            clench_gripper()
            break
        sleep(0.01)
    hand.release()


def pick_up(item):

    robot_camera_height = 1.7
    robot_joint_height = 0.9

    (
        x_error,
        y_error,
    ) = locate(item)
    while max(abs(x_error), abs(y_error)):
        x_error, y_error = look_at(item)
        sleep(0.01)
    item_dist_from_sensor = get_distance()
    neck_tilt = getattr(NT, "pos")

    item_elevation = robot_camera_height - \
        item_dist_from_sensor * np.sin(np.deg2rad(neck_tilt))
    item_dist = item_dist_from_sensor * np.cos(np.deg2rad(neck_tilt))

    desired_grab_dist = (
        0.3  # approximate distance arm will need to stretch from shoulder
    )
    shoulder_to_joint_dist = 0.4

    norm = np.sqrt(desired_grab_dist**2 + shoulder_to_joint_dist**2)
    angle_offset = np.arctan2(desired_grab_dist, shoulder_to_joint_dist)

    height_difference = item_elevation - robot_joint_height

    desired_angle = np.arccos(height_difference, norm) + angle_offset
    desired_distance = norm * np.cos(desired_angle)

    drive_distance_PID(desired_distance - item_dist)
    LF.rotate(np.rad2deg(desired_angle))

    # uhhhhhhhhhhh


def look_for(item):
    pan_speed = 1
    NT.rotate(0)
    NP.rotate(0)
    while in_motion():
        sleep(0.01)

    speak("looking for " + str(item))

    eye = cv2.VideoCapture(2)
    start_time = time.time()

    try:
        while True:
            x, y, h, w = locate(item, eye)
            while x is None:
                # Scanning for item
                x, y, h, w = locate(item, eye)
                NP.rotate(np.pi * np.sin(pan_speed *
                          (time.time() - start_time)))
                sleep(0.01)

            speak("I think I found the " + str(item))

            not_found_count = 0
            found_count = 0

            while True:
                x_error, y_error = look_at(item, eye)
                if x_error is None:
                    not_found_count += 1
                else:
                    found_count += 1

                # Avoid division by zero
                if not_found_count == 0:
                    appearance_ratio = float('inf')
                else:
                    appearance_ratio = found_count / not_found_count

                # Decide if item was lost or confirmed
                if appearance_ratio < 0.1:
                    speak("lost sight of " + str(item) + ", scanning again")
                    break  # Go back to scanning
                if appearance_ratio > 1:
                    speak("found " + str(item))
                    return  # Successful exit

                sleep(0.01)
    finally:
        eye.release()  # Always release camera on any exit


def converse(message):
    speak(chatbot.generate_response(message))


def substring_after(s, delim):
    return s.partition(delim)[2]


def level_FB():
    arduino.write(("zeroFB").encode("utf-8"))
    pitch = get_pitch()
    speak("this:" + str(pitch))
    LF.rotate(-pitch)
    while in_motion():
        sleep(0.1)
    arduino.write(("zeroFB").encode("utf-8"))
    sleep(0.1)
    LF.rotate(0)
    speak("zeroed FB")


def level_LR():
    arduino.write(("zeroLR").encode("utf-8"))
    roll = get_roll()
    speak("this:" + str(roll))
    LL.rotate(roll)
    while in_motion():
        sleep(0.1)
    arduino.write(("zeroLR").encode("utf-8"))
    sleep(0.1)
    LL.rotate(0)
    speak("zeroed LR")


###################################### THREADING ######################################

def look_at_continually(item, stop_event):
    eye = cv2.VideoCapture(2)
    while not stop_event.is_set():
        look_at(item, eye)
        sleep(0.01)
    eye.release()


def start_look_at_thread(item="person"):
    global stop_event, look_at_thread
    stop_event = threading.Event()
    look_at_thread = threading.Thread(
        target=look_at_continually, args=(item, stop_event)
    )
    look_at_thread.start()


def listen_for_stop(stop_event):
    while True:
        message = listen()
        if "stop" in message:
            stop_event.set()
            speak("stopping")
            break


def start_listen_for_stop_thread():
    global stop_event, listen_for_stop_thread
    stop_event = threading.Event()
    listen_for_stop_thread = threading.Thread(
        target=listen_for_stop, args=(stop_event,)
    )
    listen_for_stop_thread.start()


###################################### MAIN ######################################
os.system("amixer -q -M sset Headphone 100%")
speak("robot python script successfully initiated... hello")

sleep(1)

pitch0 = get_pitch()
NT.rotate(-20)
while in_motion():
    sleep(0.01)
pitch1 = get_pitch()
if not ((pitch0 + 5) < pitch1):
    speak("servos not operational... stopping script")
    # os.system("sudo shutdown -h now")
    # exit()
NT.rotate(0)
while in_motion():
    sleep(0.01)
speak("servo check completed")

sleep(1)

level_FB()
level_LR()

start_look_at_thread()

sleep(1)

try:
    while True:
        message = listen()

        ##### Diagnostics #####

        if "drive" in message:
            if "forward" in message:
                drive(50, 50)
                sleep(1)
                drive(0, 0)
            if "backward" in message:
                drive(-50, -50)
                sleep(1)
                drive(0, 0)
        elif "turn" in message:
            if "right" in message:
                drive(100, -100)
                sleep(1)
                drive(0, 0)
            if "left" in message:
                drive(-100, 100)
                sleep(1)
                drive(0, 0)
        elif "raise" in message:
            if ("left" in message) or ("both" in message):
                target_LS, target_LE, target_LW = arm.get_RK(0.35, 0)
                LS.rotate(target_LS)
                LE.rotate(target_LE)
                LW.rotate(target_LW)
            if ("right" in message) or ("both" in message):
                RS.rotate(0)
                RE.rotate(90)
                while in_motion():
                    sleep(0.01)
        elif "lower" in message:
            if ("right" in message) or ("both" in message):
                RS.rotate(-90)
                RE.rotate(0)
                RW.rotate(0)
                while in_motion():
                    sleep(0.01)
            if ("left" in message) or ("both" in message):
                LS.rotate(-90)
                LE.rotate(0)
                LW.rotate(0)
                while in_motion():
                    sleep(0.01)
        elif "look up" in message:
            NT.rotate(0)
        elif "location" in message:
            try:
                speak(
                    "my longitude is"
                    + get_longitude()
                    + "...my latitude is"
                    + get_latitude()
                )
            except TypeError:
                speak("G P S signal is blocked... lattitude and longitude unknown")
        elif "heading" in message:
            speak("my heading is" + str(get_heading()))
        elif "pitch" in message:
            speak("my pitch is" + str(get_pitch()))
        elif "roll" in message:
            speak("my pitch is" + str(get_roll()))
        elif "distance" in message:
            speak(str(get_distance()) + "meters")
        elif "level" in message:
            stop_event.set()
            look_at_thread.join()
            NP.rotate(0)
            NT.rotate(0)
            while in_motion():
                sleep(0.1)
            level_FB()
            level_LR()
            start_look_at_thread()

        ##### Calibration #####

        elif "calibrate compass" in message:
            calibrate_compass()

        ##### Complex Protocals #####

        elif "go to" in message:
            go_to(substring_after(message, "to"))
        elif "follow me" in message:
            speak("now following you")
            stop_event.set()
            look_at_thread.join()
            start_listen_for_stop_thread()
            while not stop_event.is_set():
                approach("person", 1)
            listen_for_stop_thread.join()
            start_look_at_thread()
        elif "this location is" in message:
            name_location(substring_after(message, "to"))
        elif "power off" in message:
            speak("powering off Raspberry Pi")
            os.system("sudo shutdown -h now")
        elif "reboot" in message:
            speak("rebooting Raspberry Pi")
            os.system("sudo reboot")

        elif "grab the" in message:

            stop_event.set()
            look_at_thread.join()
            item = substring_after(message, "the")
            speak("grabbing the" + str(item))

            target_LS, target_LE, target_LW = arm.get_RK(0.35, 0)
            LS.rotate(target_LS)
            LE.rotate(target_LE)
            LW.rotate(target_LW)
            while in_motion():
                sleep(0.01)
            # look_for(item)
            # approach(item, 0.5)
            # pick_up(item)
            grab("cup")
            LS.rotate(-90)
            LE.rotate(0)
            LW.rotate(0)
            while in_motion():
                sleep(0.01)
            start_look_at_thread()

        else:
            converse(message)
        sleep(0.01)
except KeyboardInterrupt:
    stop_event.set()
    look_at_thread.join()
    speak("Keyboard interrupt received... stopping Python script")
    exit()
