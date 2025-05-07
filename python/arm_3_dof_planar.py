import math
import numpy as np


class Arm_3_DOF_Planar:
    def __init__(self, L1, L2, L3):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def get_RK(self, x_target, y_target):

        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        x_wrist_target = x_target - L3

        # Calculate the distance from the base to the end-effector
        distance = math.sqrt(x_wrist_target**2 + y_target**2)

        # Check if the target point is reachable
        if (distance > L1 + L2) or (x_wrist_target < 0):
            return None

        theta2 = math.acos(
            (x_wrist_target**2 + y_target**2 - L1**2 - L2**2) / (2 * L1 * L2)
        )
        theta1 = math.atan2(y_target, x_wrist_target) - math.atan2(
            L2 * math.sin(theta2), L1 + L2 * math.cos(theta2)
        )

        theta3 = -(theta1 + theta2)

        # Convert angles to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)

        return theta1_deg, theta2_deg, theta3_deg

    def get_FK(self, theta1_deg, theta2_deg, theta3_deg):

        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        # Convert angles to radians
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)
        theta3 = np.radians(theta3_deg)

        # Calculate the coordinates of the end-effector
        x = (
            L1 * np.cos(theta1)
            + L2 * np.cos(theta1 + theta2)
            + L3 * np.cos(theta1 + theta2 + theta3)
        )
        y = (
            L1 * np.sin(theta1)
            + L2 * np.sin(theta1 + theta2)
            + L3 * np.sin(theta1 + theta2 + theta3)
        )

        return x, y
