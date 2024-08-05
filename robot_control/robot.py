"""
Program Robot Kinematics
Author: Fahmi Fathur Rohman
"""

from math import pi
import numpy as np

class robot:
    def __init__(self, GEOMETRI_ROBOT, WHEEL_RADIUS):
        # Invers dari parameter geometris robot
        bb = 1/GEOMETRI_ROBOT

        # Matriks transformasi untuk kinematika terbalik
        Ti = np.array(
            [[1, -1, -GEOMETRI_ROBOT],
             [1, 1, GEOMETRI_ROBOT],
             [1, 1, -GEOMETRI_ROBOT],
             [1, -1, GEOMETRI_ROBOT]]
        )

        # Matriks transformasi untuk kinematika maju
        Tf = np.array(
            [[1, 1, 1, 1],
             [-1, 1, 1, -1],
             [-bb, bb, -bb, bb]]
        )
        
        # Inisialisasi matriks transformasi
        self.inverse_transform_matrix = (1/WHEEL_RADIUS) * Ti
        self.forward_transform_matrix = (WHEEL_RADIUS/4) * Tf
        self.raw_position = np.zeros(4).astype(float)

    def compute_inverse_kinematic(self, input, com):
        # Menghitung kecepatan motor dari kecepatan robot
        motor_velocity = np.zeros(4).astype(int)
        robot_velocity = np.array([input[0], input[1], input[2]])
        raw_data = np.matmul(self.inverse_transform_matrix, robot_velocity)
        if com == 'RAD':
            for i in range(len(raw_data)):
                if i == 1 or i == 3:
                    motor_velocity[i] = int(raw_data[i])
                else:
                    motor_velocity[i] = -1 * int(raw_data[i])
        elif com == 'RPM':
            for i in range(len(raw_data)):
                if i == 1 or i == 3:
                    motor_velocity[i] = int(self.convert_rad_to_RPM(raw_data[i]))
                else:
                    motor_velocity[i] = -1 * int(self.convert_rad_to_RPM(raw_data[i]))
        return motor_velocity
    
    def compute_forward_kinematic(self, input):
        # Menghitung posisi robot dari kecepatan motor
        position_data = np.zeros(3)
        for i in range(len(input)):
            if i == 0 or i == 2:
                self.raw_position[i] = -1 * input[i]
            else:
                self.raw_position[i] = input[i]
        raw_data = np.matmul(self.forward_transform_matrix, self.raw_position)
        for i in range(len(raw_data)):
            position_data[i] = raw_data[i]
        return position_data

    def convert_rad_to_RPM(self, input):
        # Mengkonversi dari radian per detik ke RPM
        return input * 60 / (2 * pi)
