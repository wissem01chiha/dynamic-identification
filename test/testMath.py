import sys
import os
import unittest
import numpy as np 
src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from utils import discreteTimeIntegral

class TestDiscreteTimeIntegral(unittest.TestCase):

    def test_basic_functionality(self):
        vector = np.array([1.0, 2.0, 3.0, 4.0])
        time_step = 0.1
        expected_result = np.array([0.05, 0.25, 0.55, 0.95])
        np.testing.assert_array_almost_equal(discreteTimeIntegral(vector, time_step), expected_result)
    
    def test_time_step_validation(self):
        vector = np.array([1.0, 2.0, 3.0, 4.0])
        with self.assertRaises(ValueError):
            discreteTimeIntegral(vector, -0.1)
        with self.assertRaises(ValueError):
            discreteTimeIntegral(vector, 0)
    
    def test_empty_vector(self):
        vector = np.array([])
        time_step = 0.1
        expected_result = np.array([])
        np.testing.assert_array_almost_equal(discreteTimeIntegral(vector, time_step), expected_result)
    
    def test_single_element_vector(self):
        vector = np.array([2.0])
        time_step = 0.1
        expected_result = np.array([0.1])
        np.testing.assert_array_almost_equal(discreteTimeIntegral(vector, time_step), expected_result)

    def test_high_time_step_warning(self):
        vector = np.array([1.0, 2.0, 3.0, 4.0])
        time_step = 10
        with self.assertWarns(UserWarning):
            discreteTimeIntegral(vector, time_step)
    
    def test_small_time_step_warning(self):
        vector = np.array([1.0, 2.0, 3.0, 4.0])
        time_step = 1e-8
        with self.assertWarns(UserWarning):
            discreteTimeIntegral(vector, time_step)


if __name__ == "__main__":
    unittest.main()
