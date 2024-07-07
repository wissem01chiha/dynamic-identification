import sys
import os
import numpy as np
import unittest

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from viscoelastic import MaxwellSlip

class TestRobot(unittest.TestCase):
    def test_friction_force_not_null(self):
        model = MaxwellSlip(3,np.ones(10),[1,2,3],[0.1,0.2,0.3],1)
        F = model.computeFrictionForce()
        self.assertIsNotNone(F)
        
    def test_friction_force_shape(self):
        v= np.ones(10)
        model = MaxwellSlip(3,v,[1,2,3],[0.1,0.2,0.3],1)
        F = model.computeFrictionForce()
        self.assertEqual(F.shape,v.shape)
        

if __name__ == "__main__":
    unittest.main() 