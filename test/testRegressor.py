import sys
import os
import numpy as np
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Regressor

class TestRegressor(unittest.TestCase):
    
    def test_regressor_not_none(self):
        reg = Regressor()
        q = np.random.rand(100,7)
        W = reg.computeBasicRegressor(q,q,q)
        self.assertIsNotNone(W)
        
    def test_regressor_cond(self):
        reg = Regressor()
        
        
        
        
        
        
        
        
if __name__ == "__main__":
    unittest.main() 