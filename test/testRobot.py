import sys
import os
import numpy as np
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Robot, Regressor

class TestRobot(unittest.TestCase):
    def test_mass_matrix_is_not_none(self):
        robot = Robot()
        M = robot.massMatrix()
        self.assertIsNotNone(M, "The computed matrix is None")
        
    def test_stiffness_torques_not_none(self):
        robot = Robot()
        tau_s = robot.computeStiffnessTorques([1.25, 1, 125,1,1,1,1])
        self.assertIsNotNone(tau_s, "The computed matrix is None")
        
    def test_friction_torque_not_none(self):
        robot = Robot()
        tau_f= robot.computeFrictionTorques(np.random.rand(1001, 7),1)
        self.assertIsNotNone(tau_f, "The computed matrix is None")
        
    def test_friction_torque_size(self):
        robot = Robot()
        input_velocity = np.random.rand(1001, 7) 
        tau_f= robot.computeFrictionTorques(input_velocity,1)
        self.assertEqual(np.size(tau_f),np.size(input_velocity))
        
    def test_corlois_matrix_not_none(self):
        robot = Robot()
        C = robot.computeCorlolisMatrix()
        self.assertIsNotNone(C,"The computed matrix is None")
        
    def test_gravity_torque_not_none(self):
        robot = Robot()
        tau_g = robot.computeGravityTorques()
        self.assertIsNotNone(tau_g,"The computed vector is None")
        

if __name__ == "__main__":
    unittest.main() 
        
