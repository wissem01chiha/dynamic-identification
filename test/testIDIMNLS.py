import sys
import os
import unittest 
import numpy as np

src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from identification import IDIMNLS

class TestIDIMNLS(unittest.TestCase):
    
    @staticmethod
    def modelFun(params):
        array = np.random.normal(0, 0.5, size=(1000,10))
        for i, param in enumerate(params):
            array += param * np.sin((i+1) * array)
        return array
    
    def test_cost_function_not_none(self):
        output = 1.2*np.random.normal(0,0.5,size=(1000,10))
        model = IDIMNLS(5,output,TestIDIMNLS.modelFun)
        obj = model.computeLsCostFunction(np.random.rand(7))
        self.assertIsNotNone(obj)
        
    def test_cost_function_shape(self):
        output = 1.2*np.random.normal(0,0.5,size=(1000,10))
        model = IDIMNLS(5,output,TestIDIMNLS.modelFun)
        obj = model.computeLsCostFunction(np.random.rand(7))
        self.assertEqual(obj.ndim==0,True)
        
    def test_optimize_not_none(self):
        output = 1.2*np.random.normal(0,0.5,size=(1000,10))
        model = IDIMNLS(5,output,TestIDIMNLS.modelFun)
        sol = model.optimize(np.zeros(5))
        print(sol)
        self.assertIsNotNone(sol)
        
if __name__ == "__main__":
    unittest.main() 