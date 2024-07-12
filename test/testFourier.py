import sys
import os
import numpy as np
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from trajectory import FourierGenerator

class TestFourier(unittest.TestCase):
    
    def setUp(self) -> None:
        self.trajectory_parameters = {
        'samples': 100,
        'nbfourierterms': 5,
        'frequancy': 3,
        'ndof': 7,
        'Aij': [[np.random.randn() for _ in range(5)] for _ in range(7)],  
        'Bij': [[np.random.randn() for _ in range(5)] for _ in range(7)],
        'k1' : 2,
        'k2' :0.5  
        }
        
    def test_compute_traj_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        q, v, a =traj.computeTrajectoryState(1)
        self.assertIsNotNone(a)
        self.assertIsNotNone(v)
        self.assertIsNotNone(q)
        
    def test_traj_full_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        q0 = np.zeros(7)
        qp0 = np.zeros(7)
        qpp0 = np.zeros(7)
        q, v, a = traj.computeFullTrajectory(0, 10,q0,qp0,qpp0)
        self.assertIsNotNone(q)
        
    def test_trajectory_criterion(self):
        traj = FourierGenerator(self.trajectory_parameters)
        J = traj.computeTrajectoryCriterion(0,1,10*np.zeros(7))
        print(J)
        self.assertIsNotNone(J)
        
if __name__ == "__main__":
    unittest.main() 