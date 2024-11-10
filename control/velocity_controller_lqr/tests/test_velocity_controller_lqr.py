import numpy as np
import pytest
import rclpy
from velocity_controller_lqr.velocity_controller_lqr import LQR_controller

controller = LQR_controller(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

class TestVelocityController:
    
    
    def test_placeholder(self):
        assert controller is not None  # Simple test to ensure the controller initializes
    
    def test_ssa(self,capsys):
        print("Commencing ssa test: \n")
        
        assert controller.ssa(np.pi + 1) == -np.pi + 1
        
        assert controller.ssa(-np.pi - 1) == (np.pi - 1)
        
        print("SSA test passed")
        
        captured = capsys.readouterr()
    
    def test_quaternion_to_euler_angle(self):
        print("Commencing quaternion to euler angle test: \n")
        
        roll, pitch, yaw = controller.quaternion_to_euler_angle(0.5, 0.5, 0.5, 0.5)
        assert roll == np.pi / 2
        assert pitch == 0
        assert yaw == np.pi / 2
        
        print("Quaternion to euler angle test passed")
    
    def test_saturate(self):
        
        print("Commencing saturate test: \n")
        
        windup = False
        assert controller.saturate(10, windup, 5) == 5
        
        windup = False
        assert controller.saturate(-10, False, 5) == -5
        
        windup = True
        assert controller.saturate(10, False, 15) == 10

        windup = True
        assert controller.saturate(-10, False, 15) == -10

        print("Saturate test passed")
        
    def test_anti_windup(self):
        
        print("Commencing anti windup test: \n")
        
        windup = True
        assert controller.anti_windup(10, 5, 10, windup) == 10
        
        windup = False
        assert controller.anti_windup(1, 5, 10, windup) == 15
        
        print("Anti windup test passed")
        
    def test_final(self):
        print("¯\_(ツ)_/¯ ehh good enough pass anyway")
        pass