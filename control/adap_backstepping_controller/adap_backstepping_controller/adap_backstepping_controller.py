import numpy as np
from nav_msgs.msg import Odometry
from vortex_msgs.msg import ReferenceFilter


class AdaptiveBackstep:
    def __init__(self, K1: np.ndarray, K2: np.ndarray, M: np.ndarray) -> None:
        self.K_1 = K1
        self.K_2 = K2
        self.M = M
        self.adap_param = np.zeros((12, 1), dtype=float)
        self.d_est = np.zeros((6, 1), dtype=float)

    def control_law(self, state: Odometry, reference: ReferenceFilter) -> np.ndarray:
        """Calculates the control input based on the state and reference.

        Args:
            state (Odometry): The current state of the system.
            reference (ReferenceFilter): The reference to follow.

        Returns:
            np.ndarray: The control input.
        """
        # Transform the Odometry message to a state vector
        x, y, z, roll, pitch, yaw, u, v, w, p, q, r = self.odom_to_state(state)

        eta = np.array([x, y, z, roll, pitch, yaw]).reshape((6, 1))
        nu = np.array([u, v, w, p, q, r]).reshape((6, 1))

        # Reference
        eta_d = np.array(
            [
                reference.x,
                reference.y,
                reference.z,
                reference.roll,
                reference.pitch,
                reference.yaw,
            ]
        ).reshape((6, 1))

        r_b_bg = np.array([0.01, 0.0, 0.02]).reshape((3, 1))
        r_b_skew = self.skew_symmetric_matrix(r_b_bg)

        I_b_b = np.array([[0.68, 0.0, 0.0], [0.0, 3.32, 0.0], [0.0, 0.0, 3.34]])
        # I_b_b = np.array([[0.4278, 0.0, 0.0],
        #                  [0.0, 1.6290, 0.0],
        #                  [0.0, 0.0, 1.7899]])

        m = 30

        v2_skew = self.skew_symmetric_matrix(nu[3:])
        I_b_v2_skew = self.skew_symmetric_matrix(I_b_b @ nu[3:])

        C_RB = self.calculate_coriolis_matrix(r_b_skew, v2_skew, I_b_v2_skew, m)

        # error eta
        e_eta = eta - eta_d
        e_eta[3] = self.ssa(e_eta[3])
        e_eta[4] = self.ssa(e_eta[4])
        e_eta[5] = self.ssa(e_eta[5])

        J_eta = self.J_matrix(e_eta[3], e_eta[4], e_eta[5])

        J_eta_inv = np.linalg.inv(J_eta)

        j_dot = self.j_dot_matrix(e_eta[3], e_eta[4], e_eta[5], nu[3], nu[4], nu[5])

        # Alpha
        alpha = -J_eta_inv @ self.K_1 @ e_eta

        # Backstepping variables
        z_1 = e_eta
        z_2 = nu - alpha

        # alpha_dot
        alpha_dot = (
            J_eta_inv @ j_dot @ J_eta_inv @ self.K_1 @ e_eta - J_eta_inv @ self.K_1 @ nu
        )

        # f_eta_nu
        Y_v = np.array(
            [
                [nu[0][0], nu[0][0] * np.abs(nu[0][0]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, nu[1][0], nu[1][0] * np.abs(nu[1][0]), 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, nu[2][0], nu[2][0] * np.abs(nu[2][0]), 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, nu[3][0], nu[3][0] * np.abs(nu[3][0]), 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, nu[4][0], nu[4][0] * np.abs(nu[4][0]), 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, nu[5][0], nu[5][0] * np.abs(nu[5][0])],
            ]
        )

        # adaptive gain parameter update
        adapt_gain = np.diag(
            [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        )
        adapt_param_dot = adapt_gain @ np.transpose(Y_v) @ z_2

        adapt_d_gain = np.diag([0.3, 0.3, 0.45, 0.2, 0.7, 0.6])
        d_est_dot = adapt_d_gain @ z_2

        f_eta_nu = Y_v @ self.adap_param

        # tau
        tau = (
            C_RB @ nu
            + self.M @ alpha_dot
            - np.transpose(J_eta) @ z_1
            - self.K_2 @ z_2
            - f_eta_nu
            - self.d_est
        )

        adapt_param_dot = adapt_param_dot.reshape((12, 1))
        d_est_dot = d_est_dot.reshape((6, 1))

        # Update adaptive parameter
        self.adap_param += adapt_param_dot * 0.04
        self.d_est += d_est_dot * 0.04
        # print("Adaptive parameter: ", self.adap_param)

        # Limit adaptive parameter
        self.adap_param = np.clip(self.adap_param, -80, 80)
        self.d_est = np.clip(self.d_est, -80, 80)

        # tau limit
        tau = np.clip(tau, -80, 80)

        # tau[3] = 0
        return tau.reshape((6,))

    @staticmethod
    def calculate_coriolis_matrix(
        s_rb_g: np.ndarray, skew_v: np.ndarray, s_rb_g_v: np.ndarray, m: float
    ) -> np.ndarray:
        """Returns the Coriolis matrix times the velocity vector nu."""
        # C_RB matrix for a rigid body
        C_RB = np.block(
            [[m * skew_v, -m * skew_v * s_rb_g], [m * skew_v * s_rb_g, -s_rb_g_v]]
        )
        return C_RB

    @staticmethod
    def rotationmatrix_in_yaw_transpose(psi: float) -> np.ndarray:
        """Returns the transposed rotation matrix in the yaw angle psi."""
        R = np.array(
            [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
        )
        R_trps = np.transpose(R)
        return R_trps

    @staticmethod
    def J_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
        """Returns the J matrix."""
        cos_phi = np.cos(phi[0])
        sin_phi = np.sin(phi[0])
        cos_theta = np.cos(theta[0])
        sin_theta = np.sin(theta[0])
        cos_psi = np.cos(psi[0])
        sin_psi = np.sin(psi[0])

        R = np.array(
            [
                [
                    cos_psi * cos_theta,
                    -sin_psi * cos_phi + cos_psi * sin_theta * sin_phi,
                    sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
                ],
                [
                    sin_psi * cos_theta,
                    cos_psi * cos_phi + sin_phi * cos_theta * sin_psi,
                    -cos_psi * sin_phi + sin_psi * cos_phi * sin_theta,
                ],
                [-sin_theta, cos_theta * sin_phi, cos_theta * cos_phi],
            ]
        )

        T = np.array(
            [
                [1, sin_phi * np.tan(theta[0]), cos_phi * np.tan(theta[0])],
                [0, cos_phi, -sin_phi],
                [0, sin_phi / cos_theta, cos_phi / cos_theta],
            ]
        )

        J = np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), T]])

        return J

    @staticmethod
    def j_dot_matrix(
        phi: float, theta: float, psi: float, p: float, q: float, r: float
    ) -> np.ndarray:
        cos_phi = np.cos(phi[0])
        sin_phi = np.sin(phi[0])
        cos_theta = np.cos(theta[0])
        sin_theta = np.sin(theta[0])
        cos_psi = np.cos(psi[0])
        sin_psi = np.sin(psi[0])

        R = np.array(
            [
                [
                    cos_psi * cos_theta,
                    -sin_psi * cos_phi + cos_psi * sin_theta * sin_phi,
                    sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
                ],
                [
                    sin_psi * cos_theta,
                    cos_psi * cos_phi + sin_phi * cos_theta * sin_psi,
                    -cos_psi * sin_phi + sin_psi * cos_phi * sin_theta,
                ],
                [-sin_theta, cos_theta * sin_phi, cos_theta * cos_phi],
            ]
        )

        S = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])

        R_dot = r[0] * R @ S

        tan_theta = sin_theta / cos_theta
        inv_cos2 = 1.0 / (cos_theta * cos_theta)  # = 1/cos^2(theta)

        # Partial derivative wrt phi
        dT_dphi = np.array(
            [
                [0.0, cos_phi * tan_theta, -sin_phi * tan_theta],
                [0.0, -sin_phi, -cos_phi],
                [0.0, cos_phi / cos_theta, -sin_phi / cos_theta],
            ]
        )

        # Partial derivative wrt theta
        dT_dtheta = np.array(
            [
                [0.0, sin_phi * inv_cos2, cos_phi * inv_cos2],
                [0.0, 0.0, 0.0],
                [
                    0.0,
                    (sin_phi * sin_theta) * inv_cos2,
                    (cos_phi * sin_theta) * inv_cos2,
                ],
            ]
        )

        T_dot = dT_dphi * p[0] + dT_dtheta * q[0]

        J_dot = np.block([[R_dot, np.zeros((3, 3))], [np.zeros((3, 3)), T_dot]])

        return J_dot

    @staticmethod
    def skew_symmetric_matrix(vector: np.array) -> np.ndarray:
        """Returns the skew symmetric matrix times the angular velocity r."""
        x, y, z = vector
        return np.array([[0, -z[0], y[0]], [z[0], 0, -x[0]], [-y[0], x[0], 0]])

    @staticmethod
    def ssa(angle: float) -> float:
        """Maps an angle to the range [-pi, pi]."""
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        return angle

    @staticmethod
    def quat2euler(quaternion: np.array) -> np.array:
        """Converts a quaternion to Euler angles."""
        w, x, y, z = quaternion

        roll = np.arctan2(2 * (y * z + x * w), 1 - 2 * (x * x + y * y))
        pitch = -np.arcsin(2 * (x * z - y * w))
        yaw = np.arctan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z))

        return np.array([roll, pitch, yaw])

    @staticmethod
    def odom_to_state(msg: Odometry) -> np.array:
        """Converts an Odometry message to a state 3DOF vector.

        Args:
            msg (Odometry): The Odometry message to convert.

        Returns:
            np.ndarray: The state vector.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w,
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
        ]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = AdaptiveBackstep.quat2euler(orientation_list)

        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        w = msg.twist.twist.linear.z
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        state = np.array([x, y, z, roll, pitch, yaw, u, v, w, p, q, r])

        return state
