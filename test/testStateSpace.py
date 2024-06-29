import os 
import sys
import unittest
import numpy as np

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from dynamics import StateSpace

class TestStateSpace(unittest.TestCase):
    
    def test_state_matrices_not_none(self):
        model = StateSpace()
        A, B, C, D = model.computeStateMatrices(np.random.rand(7),np.random.rand(7))
        self.assertIsNotNone(A)
        self.assertIsNotNone(B)
        self.assertIsNotNone(C)
        
    def test_state_matrices_shape(self):
        model = StateSpace()
        A, B, C, D = model.computeStateMatrices(np.random.rand(7),np.random.rand(7))
        self.assertEqual(A.shape, (14,14))


if __name__ == "__main__":
    unittest.main() 
        