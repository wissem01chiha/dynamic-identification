import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from trajectory import FourierGenerator

class TestFourier(unittest.TestCase):
    
    def setUp(self) -> None:
        np.random.seed()
        self.trajectory_parameters = {
        'samples': 120,
        'nbfourierterms': 5,
        'frequancy': 1,
        'ndof': 7,
        'Aij': [[np.random.randn() for _ in range(5)] for _ in range(7)],  
        'Bij': [[np.random.randn() for _ in range(5)] for _ in range(7)] ,
        'k1' : 1,
        'k2': 1
        }
        
    def test_compute_traj_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        q0 = np.zeros(7)
        qp0 = np.zeros(7)
        qpp0 = np.zeros(7)
        state =traj.computeTrajectoryState(1.2,q0,qp0,qpp0)
        self.assertIsNotNone(state)
        
    def test_traj_full_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        q0 = np.random.rand(7)
        qp0 = np.zeros(7)
        qpp0 = np.zeros(7)
        Q, Qp, Qpp = traj.computeFullTrajectory(0,100,q0,qp0,qpp0)
        self.assertIsNotNone(Q)
        self.assertIsNotNone(Qp)
        self.assertIsNotNone(Qpp)
        
    def test_traj_criteria_not_none(self):
        traj =  FourierGenerator(self.trajectory_parameters)
        q0 =np.random.rand(7)
        J = traj.computeTrajectoryCriterion(0,100,q0,q0,q0)
        self.assertIsNotNone(J)
        
    def test_traj_visualize(self):
        traj = FourierGenerator(self.trajectory_parameters)
        q0 = np.random.rand(7)
        #traj.visualizeTrajectory(0,1,q0,q0,q0)
        #plt.show()
        
        
    def test_traj_identificatbility(self):
        traj = FourierGenerator(self.trajectory_parameters)
        torque = np.random.rand(self.trajectory_parameters['samples'],7)
        q0 = np.zeros(7)
        qp0 = np.zeros(7)
        qpp0 =np.zeros(7)
        x = np.abs(np.random.rand(98))
        Q, Qp, Qpp = traj.computeFullTrajectory(0,1,q0,qp0,qpp0)
        traj.visualizeTrajectoryIdentifiability(0,1, torque, Q, Qp, Qpp, x)
        plt.show()
        
if __name__ == "__main__":
    unittest.main() 