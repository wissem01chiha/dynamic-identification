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
        'frequancy': 300,
        'ndof': 7,
        'Aij': [[np.random.randn() for _ in range(5)] for _ in range(7)],  
        'Bij': [[np.random.randn() for _ in range(5)] for _ in range(7)]   
        }
        
    def test_compute_traj_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        state =traj.computeTrajectoryStates(1)
        self.assertIsNotNone(state)
        
    def test_traj_full_state_not_none(self):
        traj = FourierGenerator(self.trajectory_parameters)
        state =traj.computeTrajectoryStates(1)
        
if __name__ == "__main__":
    unittest.main() 