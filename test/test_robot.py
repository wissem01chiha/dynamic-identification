import sys
import os
import numpy as np
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pyDynaMapp.dynamics import Robot

class TestRobot(unittest.TestCase):
    
    def setUp(self) -> None:
        base_dir = os.getcwd()
        pkg_dir = os.path.join(base_dir,'pyDynaMapp')
        self.urdf_file_path = os.path.join(pkg_dir,"robot/kinova/gen3.urdf")
        self.config_file_path = os.path.join(pkg_dir,"robot/kinova/config.yml")
        self.robot = Robot(self.urdf_file_path,self.config_file_path)
    
    def test_mass_matrix_is_not_none(self):
        M = self.robot.computeMassMatrix()
        self.assertIsNotNone(M, "The computed matrix is None")
        
    def test_stiffness_torques_not_none(self):
        tau_s = self.robot.computeStiffnessTorques([1.25, 1, 125,1,1,1,1])
        self.assertIsNotNone(tau_s, "The computed matrix is None")
       
    def test_friction_torque_not_none(self):
        tau_f= self.robot.computeFrictionTorques(np.random.rand(100,7),\
            np.random.rand(100,7))
        self.assertIsNotNone(tau_f, "The computed matrix is None")

    def test_friction_torque_size(self):
        tau_f= self.robot.computeFrictionTorques(np.random.rand(1001, 7),\
            np.random.rand(1001,7))
        self.assertEqual(np.size(tau_f),np.size(np.random.rand(1001, 7) ))
    
    def test_corlois_matrix_not_none(self):
        C = self.robot.computeCorlolisMatrix()
        self.assertIsNotNone(C,"The computed matrix is None")
        
    def test_gravity_torque_not_none(self):
        tau_g = self.robot.computeGravityTorques()
        self.assertIsNotNone(tau_g)
        
    def test_gravity_torque_shape(self):
        q = np.ones(7)
        tau_g = self.robot.computeGravityTorques(q)
        self.assertEqual(tau_g.shape,q.shape)
        
    def test_inertia_params_not_none(self):
        Xhi = self.robot.genralizedInertiasParams()
        self.assertIsNotNone(Xhi)
    
    def test_inertia_params_size(self):
        Xhi = self.robot.genralizedInertiasParams()
        self.assertEqual(np.size(Xhi),13*self.robot.model.nq)
        
    def test_update_inertia_params(self):
        Xhi = self.robot.genralizedInertiasParams()
        self.robot.updateInertiaParams(np.random.rand(13*self.robot.model.nq))
        Xhi_1 = self.robot.genralizedInertiasParams()
        self.assertEqual(np.any(Xhi!=Xhi_1),True,"The inertia parameters were not updated.")
    
    def test_update_stiffness_params(self):
        q = np.ones(7)
        tau_s1 = self.robot.computeStiffnessTorques(q)
        self.robot.updateStiffnessParams(np.random.rand(7))
        tau_s2 = self.robot.computeStiffnessTorques(q)
        self.assertEqual(np.any(tau_s1!=tau_s2),True)
    
    def test_actuator_torques_not_none(self):
        tau_m = self.robot.computeActuatorTorques(np.random.rand(100,7),\
            np.random.rand(100,7),np.random.rand(100,7))
        self.assertIsNotNone(tau_m)
        
    def test_actuator_torques_shape(self):
        tau_m = self.robot.computeActuatorTorques(np.random.rand(100,7),\
        np.random.rand(100,7),np.random.rand(100,7))
        self.assertEqual(tau_m.shape==(100,7),True)
        
    def test_actuator_torques_params(self):
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        qpp = np.random.rand(100,7)
        tau_m_1 = self.robot.computeActuatorTorques(q, qp, qpp)
        self.robot.updateActuatorParams(np.random.rand(70,1))
        tau_m_2 = self.robot.computeActuatorTorques(q, qp, qpp)
        self.assertEqual(np.any(tau_m_1!=tau_m_2), True)
        
    def test_update_friction_params(self):
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        tau_f1 = self.robot.computeFrictionTorques(qp,q)
        self.robot.updateFrictionParams(np.random.rand(35))
        tau_f2 = self.robot.computeFrictionTorques(qp,q)
        self.assertEqual(np.any(tau_f1!=tau_f2), True)
        
    def test_identification_model_not_none(self):
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        qpp = np.random.rand(100,7)
        x = np.random.rand(209)
        self.robot.setIdentificationModelData(q,qp,qpp)
        tau = self.robot.computeIdentificationModel(x)
        self.assertIsNotNone(tau)
        
    def test_identification_model_shape(self):
        q = np.random.rand(100,7)
        qp = np.random.rand(100,7)
        qpp = np.random.rand(100,7)
        x = np.random.rand(209)
        self.robot.setIdentificationModelData(q,qp,qpp)
        tau = self.robot.computeIdentificationModel(x)
        self.assertEqual(tau.shape,(100,7))
        
if __name__ == "__main__":
    unittest.main() 