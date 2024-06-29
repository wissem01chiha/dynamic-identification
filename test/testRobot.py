import sys
import os
import numpy as np
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Robot

class TestRobot(unittest.TestCase):
    def test_mass_matrix_is_not_none(self):
        robot = Robot()
        M = robot.computeMassMatrix()
        self.assertIsNotNone(M, "The computed matrix is None")
        
    def test_stiffness_torques_not_none(self):
        robot = Robot()
        tau_s = robot.computeStiffnessTorques([1.25, 1, 125,1,1,1,1])
        self.assertIsNotNone(tau_s, "The computed matrix is None")
        
    def test_friction_torque_not_none(self):
        robot = Robot()
        tau_f= robot.computeFrictionTorques(np.random.rand(1001))
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
        
    def test_inertia_params_not_none(self):
        robot = Robot()
        Xhi = robot.genralizedInertiasParams()
        self.assertIsNotNone(Xhi,"The computed vector is None")
    
    def test_inertia_params_size(self):
        robot = Robot()
        Xhi = robot.genralizedInertiasParams()
        self.assertEqual(np.size(Xhi),13*robot.model.nq)
        
    def test_update_inertia_params(self):
        robot = Robot()
        Xhi = robot.genralizedInertiasParams()
        robot.updateInertiaParams(np.random.rand(13*robot.model.nq))
        Xhi_1 = robot.genralizedInertiasParams()
        self.assertEqual(np.any(Xhi!=Xhi_1),True,"The inertia parameters were not updated.")
    
    def test_update_stiffness_params(self):
        robot = Robot()
        s1 = robot.updateStiffnessParams(np.random.rand(7))
        self.assertEqual(np.any(s1!=s2),True)
        
if __name__ == "__main__":
    unittest.main() 
        
