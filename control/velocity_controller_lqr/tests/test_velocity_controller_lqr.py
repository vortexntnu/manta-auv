import numpy as np
from velocity_controller_lqr.velocity_controller_lqr_lib import (
    LQRController,
    LQRParameters,
)

lqr_params = LQRParameters()
controller = LQRController(lqr_params, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))


class TestVelocityController:
    def test_placeholder(self):
        assert controller is not None

    def test_ssa(self):
        print("Commencing ssa test: \n")

        assert LQRController.ssa(np.pi + 1) == -np.pi + 1

        assert LQRController.ssa(-np.pi - 1) == (np.pi - 1)

        print("SSA test passed")

    def test_quaternion_to_euler_angle(self):
        print("Commencing quaternion to euler angle test: \n")

        roll, pitch, yaw = LQRController.quaternion_to_euler_angle(0.5, 0.5, 0.5, 0.5)
        assert roll == np.pi / 2
        assert pitch == 0
        assert yaw == np.pi / 2

        print("Quaternion to euler angle test passed")

    def test_saturate(self):
        print("Commencing saturate test: \n")

        # Test case 1: Saturation occurs
        windup, saturated_value = controller.saturate(10, False, 5)
        assert saturated_value == 5.0
        assert windup == True

        # Test case 2: Saturation occurs with negative limit
        windup, saturated_value = controller.saturate(-10, False, 5)
        assert saturated_value == -5.0
        assert windup == True

        # Test case 3: No saturation
        windup, saturated_value = controller.saturate(3, True, 5)
        assert saturated_value == 3.0
        assert windup == False

        # Test case 4: No saturation with negative value
        windup, saturated_value = controller.saturate(-3, True, 5)
        assert saturated_value == -3.0
        assert windup == False

        print("Saturate test passed")

    def test_anti_windup(self):
        print("Commencing anti windup test: \n")

        windup = True
        assert controller.anti_windup(10, 5, 10, windup) == 10

        windup = False
        assert controller.anti_windup(1, 5, 10, windup) == 15

        print("Anti windup test passed")

    def test_max_force(self):
        assert (
            0 <= controller.max_force <= 99.9
        ), "Max force must be in the range [0, 99.9]."
