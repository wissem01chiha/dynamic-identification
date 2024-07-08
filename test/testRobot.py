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
        tau_f= robot.computeFrictionTorques(np.random.rand(100,7),np.random.rand(100,7))
        self.assertIsNotNone(tau_f, "The computed matrix is None")

    def test_friction_torque_size(self):
        robot = Robot()
        tau_f= robot.computeFrictionTorques(np.random.rand(1001, 7) ,np.random.rand(1001,7))
        self.assertEqual(np.size(tau_f),np.size(np.random.rand(1001, 7) ))
    
    def test_corlois_matrix_not_none(self):
        robot = Robot()
        C = robot.computeCorlolisMatrix()
        self.assertIsNotNone(C,"The computed matrix is None")
        
    def test_gravity_torque_not_none(self):
        robot = Robot()
        tau_g = robot.computeGravityTorques()
        self.assertIsNotNone(tau_g,"The computed vector is None")
        
    def test_gravity_torque_shape(self):
        robot = Robot()
        
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
        q = np.ones(7)
        tau_s1 = robot.computeStiffnessTorques(q)
        robot.updateStiffnessParams(np.random.rand(7))
        tau_s2 = robot.computeStiffnessTorques(q)
        self.assertEqual(np.any(tau_s1!=tau_s2),True)
    
    def test_actuator_torques_not_none(self):
        robot = Robot()
        tau_m = robot.computeActuatorTorques(np.random.rand(100,7),\
            np.random.rand(100,7),np.random.rand(100,7))
        self.assertIsNotNone(tau_m)
        
    def test_actuator_torques_shape(self):
        robot = Robot()
        tau_m = robot.computeActuatorTorques(np.random.rand(100,7),\
        np.random.rand(100,7),np.random.rand(100,7))
        self.assertEqual(tau_m.shape==(100,7),True)
        
    def test_actuator_torques_params(self):
        robot = Robot()
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        qpp = np.random.rand(100,7)
        tau_m_1 = robot.computeActuatorTorques(q, qp, qpp)
        robot.updateActuatorParams(np.random.rand(70,1))
        tau_m_2 = robot.computeActuatorTorques(q, qp, qpp)
        self.assertEqual(np.any(tau_m_1!=tau_m_2), True)
        
    def test_update_friction_params(self):
        robot = Robot()
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        tau_f1 = robot.computeFrictionTorques(qp,q)
        robot.updateFrictionParams(np.random.rand(10))
        tau_f2 = robot.computeFrictionTorques(qp,q)
        self.assertEqual(np.any(tau_f1!=tau_f2), True)
        
    def test_identification_model_not_none(self):
        robot = Robot()
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        qpp = np.random.rand(100,7)
        
if __name__ == "__main__":
    unittest.main() 
        
