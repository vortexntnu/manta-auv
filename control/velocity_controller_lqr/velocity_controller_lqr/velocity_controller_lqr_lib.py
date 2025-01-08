from dataclasses import dataclass

import numpy as np

import control as ct


@dataclass
class State:
    """Dataclass to store the state values for the LQR controller.

    Attributes:
        surge: float Surge state value
        pitch: float: Pitch state value
        yaw: float: Yaw state value
        integral_surge: float: Surge integral state value
        integral_pitch: float: Pitch integral state value
        integral_yaw: float: Yaw integral state value
    """

    surge: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    integral_surge: float = 0.0
    integral_pitch: float = 0.0
    integral_yaw: float = 0.0


@dataclass
class GuidanceValues:
    """Dataclass to store the guidance values for the LQR controller.

    Attributes:
        surge: float: Surge guidance value
        pitch: float: Pitch guidance value
        yaw: float: Yaw guidance value
        integral_surge: float: Surge integral guidance value
        integral_pitch: float: Pitch integral guidance value
        integral_yaw: float: Yaw integral guidance value
    """

    surge: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    integral_surge: float = 0.0
    integral_pitch: float = 0.0
    integral_yaw: float = 0.0


@dataclass
class LQRParameters:
    """Dataclass to store the parameters for the LQR controller.

    Attributes:
        q_surge: float: Surge state weight
        q_pitch: float: Pitch state weight
        q_yaw: float: Yaw state weight
        r_surge: float: Surge input weight
        r_pitch: float: Pitch input weight
        r_yaw: float: Yaw input weight
        i_surge: float: Surge integral weight
        i_pitch: float: Pitch integral weight
        i_yaw: float: Yaw integral weight
        i_weight: float: Integral weight
        max_force: float: Maximum force
    """

    q_surge: float = 0.0
    q_pitch: float = 0.0
    q_yaw: float = 0.0
    r_surge: float = 0.0
    r_pitch: float = 0.0
    r_yaw: float = 0.0
    i_surge: float = 0.0
    i_pitch: float = 0.0
    i_yaw: float = 0.0
    i_weight: float = 0.0
    max_force: float = 0.0
    operation_mode: str = "xbox mode"
    killswitch: bool = True


class LQRController:
    def __init__(self, parameters: LQRParameters, inertia_matrix: np.array) -> None:
        self.set_params(parameters)
        self.set_matrices(inertia_matrix)

    @staticmethod
    def quaternion_to_euler_angle(w: float, x: float, y: float, z: float) -> tuple:
        """Function to convert quaternion to euler angles.

        Parameters:
            w: float: w component of quaternion
            x: float: x component of quaternion
            y: float: y component of quaternion
            z: float: z component of quaternion

        Returns:
            phi: float: roll angle
            theta: float: pitch angle
            psi: float: yaw angle

        """
        y_square = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y_square)
        phi = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        theta = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y_square + z * z)
        psi = np.arctan2(t3, t4)

        return phi, theta, psi

    @staticmethod
    def ssa(angle: float) -> float:
        """Function to convert the angle to the smallest signed angle.

        Parameters:
            angle: float: angle in radians

        Returns:
            angle: float: angle in radians

        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def saturate(self, value: float, windup: bool, limit: float) -> tuple:
        """Function to saturate the value within the limits, and set the windup flag.

        Parameters:
            value: float: value to be saturated
            windup: bool: windup variable
            limit: float: limit for saturation

        Returns:
            (value, windup): tuple: saturated value and windup flag

        """
        if abs(value) > limit:
            windup = True
            value = limit * (value / abs(value))
        else:
            windup = False

        return windup, value

    @staticmethod
    def anti_windup(
        k_i: float, error: float, integral_sum: float, windup: bool
    ) -> float:
        """Function to saturate the integral value within the limits.

        Parameters:
            k_i : float: integral gain
            error: float: error value
            integral_sum: float: integral sum
            windup: bool: windup variable

        Returns:
            value: float: saturated value

        """
        if not windup:
            integral_sum += k_i * error

        return integral_sum

    @staticmethod
    def calculate_coriolis_matrix(
        pitch_rate: float, yaw_rate: float, sway_vel: float, heave_vel: float
    ) -> np.array:
        """Calculates the 3x3 coriolis matrix.

        Parameters:
            pitch_rate: float: pitch rate in rad/s
            yaw_rate: float: yaw rate in rad/s
            sway_vel: float: sway velocity in m/s
            heave_vel: float: heave velocity in m/s

        Returns:
            C: np.array: 3x3 Coriolis Matrix

        """
        return np.array(
            [
                [0.2, -30 * sway_vel * 0.01, -30 * heave_vel * 0.01],
                [30 * sway_vel * 0.01, 0, 1.629 * pitch_rate],
                [30 * heave_vel * 0.01, 1.769 * yaw_rate, 0],
            ]
        )

    def set_params(self, parameters: LQRParameters) -> None:
        """Sets the parameters for the LQR controller.

        Parameters:
            parameters: LQRParameters: dataclass containing the parameters for the LQR controller

        """
        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False
        self.pitch_windup = False
        self.yaw_windup = False

        self.q_surge = parameters.q_surge
        self.q_pitch = parameters.q_pitch
        self.q_yaw = parameters.q_yaw

        self.r_surge = parameters.r_surge
        self.r_pitch = parameters.r_pitch
        self.r_yaw = parameters.r_yaw

        self.i_surge = parameters.i_surge
        self.i_pitch = parameters.i_pitch
        self.i_yaw = parameters.i_yaw
        self.i_weight = parameters.i_weight
        self.max_force = parameters.max_force

        self.operation_mode = parameters.operation_mode
        self.killswitch = parameters.killswitch

    def set_matrices(self, inertia_matrix: np.array) -> None:
        """Adjusts the matrices for the LQR controller.

        Parameters:
            inertia_matrix: np.array: 3x3 inertia matrix
        """
        self.inertia_matrix_inv = np.linalg.inv(inertia_matrix)
        self.state_weight_matrix = np.block(
            [
                [np.diag([self.q_surge, self.q_pitch, self.q_yaw]), np.zeros((3, 3))],
                [
                    np.zeros((3, 3)),
                    np.diag([self.i_weight, self.i_weight, self.i_weight]),
                ],
            ]
        )

        self.input_weight_matrix = np.diag([self.r_surge, self.r_pitch, self.r_yaw])

    def update_augmented_matrices(self, coriolis_matrix: np.array) -> None:
        """Updates the augmented matrices for the LQR controller.

        Parameters:
            coriolis_matrix: np.array: 3x3 Coriolis Matrix
        """
        system_matrix = self.inertia_matrix_inv @ coriolis_matrix
        input_matrix = self.inertia_matrix_inv

        self.augmented_system_matrix = np.block(
            [[system_matrix, np.zeros((3, 3))], [-np.eye(3), np.zeros((3, 3))]]
        )
        self.augmented_input_matrix = np.block([[input_matrix], [np.zeros((3, 3))]])

    def update_error(self, guidance_values: GuidanceValues, states: State) -> np.array:
        """Updates the error values for the LQR controller.

        Parameters:
            guidance_values: GuidanceValues: 6x1 dataclass containing the guidance values
            states: State: 6x1 dataclass containing the state values

        Returns:
            state_error: np.array: 6x1 array of the state errors
        """
        surge_error = (
            guidance_values.surge - states.surge
        )  # Surge error isn't an angle, no need for angle wrapping
        pitch_error = self.ssa(guidance_values.pitch - states.pitch)
        yaw_error = self.ssa(guidance_values.yaw - states.yaw)

        # Update the running integrator sums
        self.integral_error_surge = self.anti_windup(
            self.i_surge, surge_error, self.integral_error_surge, self.surge_windup
        )
        self.integral_error_pitch = self.anti_windup(
            self.i_pitch, pitch_error, self.integral_error_pitch, self.pitch_windup
        )
        self.integral_error_yaw = self.anti_windup(
            self.i_yaw, yaw_error, self.integral_error_yaw, self.yaw_windup
        )

        state_error = np.array(
            [
                -surge_error,
                -pitch_error,
                -yaw_error,
                self.integral_error_surge,
                self.integral_error_pitch,
                self.integral_error_yaw,
            ]
        )
        return state_error

    def saturate_input(self, u: np.array) -> np.array:
        """Saturates the control input within the limits, and sets the windup flag.

        Parameters:
            u: np.array: 3x1 control input

        Returns:
            u: np.array: 3x1 saturated control input
        """
        self.surge_windup, force_x = self.saturate(
            u[0], self.surge_windup, self.max_force
        )
        self.pitch_windup, torque_y = self.saturate(
            u[1], self.pitch_windup, self.max_force
        )
        self.yaw_windup, torque_z = self.saturate(u[2], self.yaw_windup, self.max_force)

        return [force_x, torque_y, torque_z]

    def calculate_lqr_u(
        self, coriolis_matrix: np.array, states: State, guidance_values: GuidanceValues
    ) -> np.array:
        """Calculates the control input using the control library in python.

        Parameters:
            C: np.array: 3x3 Coriolis Matrix
            states: State: 6x1 dataclass containing the state values
            guidance_values: GuidanceValues: 6x1 dataclass containing the guidance values

        Returns:
            u: np.array: 3x1 control input

        """
        self.update_augmented_matrices(coriolis_matrix)

        lqr_gain, _, _ = ct.lqr(
            self.augmented_system_matrix,
            self.augmented_input_matrix,
            self.state_weight_matrix,
            self.input_weight_matrix,
        )

        state_error = self.update_error(guidance_values, states)
        u = self.saturate_input(-lqr_gain @ state_error)

        return u

    def reset_controller(self) -> None:
        """Resets the controller to the initial state."""
        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False
        self.pitch_windup = False
        self.yaw_windup = False


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
