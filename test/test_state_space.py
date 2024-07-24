import os 
import sys
import unittest
import numpy as np
import matplotlib.pyplot as plt

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from dynamics import StateSpace

class TestStateSpace(unittest.TestCase):
    
    def test_state_matrices_not_none(self):
        model = StateSpace()
        A, B, C, D = model.computeStateMatrices(np.random.rand(14))
        self.assertIsNotNone(A)
        self.assertIsNotNone(B)
        self.assertIsNotNone(C)
        
    def test_state_matrices_shape(self):
        model = StateSpace()
        A, B, C, D = model.computeStateMatrices(np.random.rand(14))
        self.assertEqual(A.shape, (14,14))
        self.assertEqual(B.shape, (14,7))
        self.assertEqual(C.shape, (7,14))
        self.assertEqual(D.shape, (7,7))
        
    def test_update_state_vector(self):
        model = StateSpace()
        x_k_1 = model.updateStateVector(np.random.rand(14),np.random.rand(7))
        self.assertIsNotNone(x_k_1)
        self.assertEqual(x_k_1.size,14)
        self.assertEqual(np.all(x_k_1.shape ==(14,)),True)
        
    def test_simulate(self):
        model = StateSpace()
        input_torques = 0.02*np.ones((20,7))
        states = model.simulate(0.5*np.ones((14,)),input_torques,'gaussian', True)
        self.assertIsNotNone(states)
        self.assertEqual(states.shape,(14,20), True)
        
    def test_state_input_vector(self):
        model = StateSpace()
        
    def test_visualize_pols_plot(self):
        model = StateSpace()
        model.visualizeStatePoles(np.random.rand(7),np.random.rand(7))
        plt.show()
        
    def test_visulize_root_locus(self):
        model = StateSpace()
        model.visualizeRootLocus(2.8*np.ones(7),8*np.ones(7))
        plt.show()
    
if __name__ == "__main__":
    unittest.main() 
        