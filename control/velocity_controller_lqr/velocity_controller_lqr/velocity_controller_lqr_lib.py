#!/usr/bin/env python3
import math as math

import control as ct
import numpy as np


class LQR_controller():
    def __init__(self, Q_surge, Q_pitch, Q_yaw, R_surge, R_pitch, R_yaw, I_surge, I_pitch, I_yaw, max_force):

        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False  # Windup variable
        self.pitch_windup = False  # Windup variable
        self.yaw_windup = False  # Windup variable
        
        self.max_force = max_force  # Maximum force that can be applied
        
        self.I_surge = I_surge  # Integral gain for surge
        self.I_pitch = I_pitch  # Integral gain for pitch
        self.I_yaw = I_yaw  # Integral gain for yaw
        
        self.Q_surge = Q_surge  # Surge cost
        self.Q_pitch = Q_pitch  # Pitch cost
        self.Q_yaw = Q_yaw  # Yaw cost
        
        self.R_surge = R_surge  # Surge control cost
        self.R_pitch = R_pitch  # Pitch control cost
        self.R_yaw = R_yaw  # Yaw control cost
        
        # VARIABLES
        self.M_inv = np.linalg.inv(np.array(
            [[30, 0.6, 0],
             [0.6, 1.629, 0],
             [0, 0, 1.769]]
        ))  # mass matrix
        
        self.Q = np.block(
            [
                [np.diag([self.Q_surge, self.Q_pitch, self.Q_yaw]), np.zeros((3, 3))],
                [np.zeros((3, 3)), np.diag([0.5, 0.5, 0.5])]
            ]
        )  # Augmented state cost matrix
        self.R = np.diag([self.R_surge, self.R_pitch, self.R_yaw])  # control cost matrix
        
    @staticmethod
    def quaternion_to_euler_angle(w: float, x: float, y: float, z: float) -> tuple:
        """
        Function to convert quaternion to euler angles.

        Args:
            w: float: w component of quaternion
            x: float: x component of quaternion
            y: float: y component of quaternion
            z: float: z component of quaternion

        Returns:
            X: float: roll angle
            Y: float: pitch angle
            Z: float: yaw angle

        """
        y_square = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y_square)
        X = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y_square + z * z)
        Z = np.arctan2(t3, t4)

        return X, Y, Z

    @staticmethod
    def ssa(angle: float) -> float:
        """
        Function to convert the angle to the smallest signed angle

        Args:
            angle: float: angle in radians

        Returns:
            angle: float: angle in radians

        """
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def saturate(self, value: float, windup: bool, limit: float) -> float:
        """
        Function to saturate the value within the limits, and set the windup flag

        Args:
            value: float: value to be saturated
            windup: bool: windup variable
            limit: float: limit for saturation

        Returns:
            value: float: saturated value

        """
        if abs(value) > limit:
            windup = True
            value = limit * value / abs(value)
        else:
            windup = False
        
        return value

    @staticmethod
    def anti_windup(K_i: float, error: float, integral_sum: float, windup: bool) -> float:
        """
        Function to saturate the integral value within the limits

        Args:
            value: float: value to be saturated
            limit: float: limit for saturation

        Returns:
            value: float: saturated value

        """
        if windup:
            integral_sum += 0.0
        else:
            integral_sum += K_i * error
        
        return integral_sum

    def calculate_coriolis_matrix(self, pitch_rate: float, yaw_rate: float, sway: float, heave: float) -> np.array:
        """
        Calculates the 3x3 coriolis matrix
        
        Parameters:
            pitch_rate: float: pitch rate in rad/s
            yaw_rate: float: yaw rate in rad/s
            sway: float: sway velocity in m/s
            heave: float: heave velocity in m/s

        Returns:
            C: np.array: 3x3 Coriolis Matrix
            
        """
        return np.array(
            [
                [0.2, -30 * sway * 0.01, -30 * heave * 0.01],
                [30 * sway * 0.01, 0, 1.629 * pitch_rate],
                [30 * heave * 0.01, 1.769 * yaw_rate, 0],
            ]
        )
        
    def calculate_lqr_u(self, C: np.array, states: np.array, guidance_values: np.array) -> np.array:
        """
        Calculates the control input using the control library in python
        
        Parameters:
            C: np.array: 3x3 Coriolis Matrix
            states: np.array: 6x1 state vector
            guidance_values: np.array: 6x1 guidance vector
        
        Returns:
            u: np.array: 3x1 control input
            
        """
        
        A = self.M_inv @ C
        B = self.M_inv

        # Augment the A and B matrices for integrators for surge, pitch, and yaw
        self.A_aug = np.block(
            [[A, np.zeros((3, 3))],
             [-np.eye(3), np.zeros((3, 3))]]
        )  # Integrators added for surge, pitch, and yaw
        self.B_aug = np.block(
            [[B], 
             [np.zeros((3, 3))]]
        )  # Control input does not affect integrators directly

        # CT LQR controller from control library python
        K, S, E = ct.lqr(
            self.A_aug, self.B_aug, self.Q, self.R
        )
        
        # Calculate the control input
        surge_error = (
            (guidance_values[0] - states[0])
        )  # Surge error no need for angle wrapping
        pitch_error = self.ssa(
            guidance_values[1] - states[1]
        )  # Apply ssa to pitch error
        yaw_error = self.ssa(
            guidance_values[2] - states[2]
        )  # Apply ssa to yaw error

        # Update the integrators for surge, pitch, and yaw
        self.integral_error_surge = self.anti_windup(self.I_surge, surge_error, self.integral_error_surge, self.surge_windup)
        self.integral_error_pitch = self.anti_windup(self.I_pitch, pitch_error, self.integral_error_pitch, self.pitch_windup)
        self.integral_error_yaw = self.anti_windup(self.I_yaw, yaw_error, self.integral_error_yaw, self.yaw_windup)
        
        state_error = np.array([-surge_error,-pitch_error,-yaw_error, self.integral_error_surge, self.integral_error_pitch, self.integral_error_yaw])

        # Augmented state vector including integrators
        u = -K @ state_error
        
        force_x = self.saturate(u[0], self.surge_windup, self.max_force)
        torque_y = self.saturate(u[1], self.pitch_windup, self.max_force)
        torque_z = self.saturate(u[2], self.yaw_windup, self.max_force)
        
        u = np.array([force_x, torque_y, torque_z])
        
        return u
        
# ---------------------------------------------------------------Main Function---------------------------------------------------------------

    
#                     .--------------.
#                 .---'  o        .    `---.
#              .-'    .    O  .         .   `-.
#           .-'     @@@@@@       .             `-.
#         .'@@   @@@@@@@@@@@       @@@@@@@   .    `.
#       .'@@@  @@@@@@@@@@@@@@     @@@@@@@@@         `.
#      /@@@  o @@@@@@@@@@@@@@     @@@@@@@@@     O     \
#     /        @@@@@@@@@@@@@@  @   @@@@@@@@@ @@     .  \
#    /@  o      @@@@@@@@@@@   .  @@  @@@@@@@@@@@     @@ \
#   /@@@      .   @@@@@@ o       @  @@@@@@@@@@@@@ o @@@@ \
#  /@@@@@                  @ .      @@@@@@@@@@@@@@  @@@@@ \
#  |@@@@@    O    `.-./  .        .  @@@@@@@@@@@@@   @@@  |
# / @@@@@        --`-'       o        @@@@@@@@@@@ @@@    . \
# |@ @@@@ .  @  @    `    @            @@      . @@@@@@    |
# |   @@                         o    @@   .     @@@@@@    |
# |  .     @   @ @       o              @@   o   @@@@@@.   |
# \     @    @       @       .-.       @@@@       @@@      /
#  |  @    @  @              `-'     . @@@@     .    .    |
#  \ .  o       @  @@@@  .              @@  .           . /
#   \      @@@    @@@@@@       .                   o     /
#    \    @@@@@   @@\@@    /        O          .        /
#     \ o  @@@       \ \  /  __        .   .     .--.  /
#      \      .     . \.-.---                   `--'  /
#       `.             `-'      .                   .'
#         `.    o     / | `           O     .     .'
#           `-.      /  |        o             .-'
#              `-.          .         .     .-'
#                 `---.        .       .---'
#                      `--------------'