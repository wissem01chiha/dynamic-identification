import logging
import numpy as np
from scipy.optimize import  least_square, curve_fit 
from utils import RMSE, cumulativeRMSE 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IDIMNLS:
    """
    Inverse Dynamics Identification Methods with Non Linear Least Square Alogrithms.
    The identification problem is formulated in a non linear optimisation problem :
    
        Xopt = argmin(X) ||IDM(q, qdot, qddot, X)- tau||
        
    Args:
        - nVars                  : number of optimization variables in the X vector.
        - output                 : desired output vector (tau)
        - computeInverseDynamics : function that retun the model computed torques.
    """
    def __init__(self,nVars,output,computeInverseDynamics,time_step=0.001) -> None:
        self.time_step  = 0.001
        self.torque     = output
        self.nVars      = nVars 
        self.inverseDynamics = computeInverseDynamics
       
    def __str__(self) -> str:
        return (f"IDIMNLS Model with {self.nVars} optimization variables,"
                f"output shape: {self.torque.shape}, "
                f"time step: {self.time_step}")
        
    def computeCostFunction(self, x:np.ndarray):
        """
        The object function to be minimized.
    
        Returns:
            - cost : (float)
        """
        assert(x.size == self.nVars)
        assert (np.ndim(self.torque) == 2)
        tau_s = self.inverseDynamics(x)
        rmse = RMSE(self.torque, tau_s) 
        cost = np.mean(rmse**2,axis=1)
        return cost 

    def evaluate(self):
         """Evaluate the model's performance using the current parameters."""
    
    def optimize(self, x0:np.ndarray, method='least_square'):
        """Optimize the cost function with NLS alorithms to mimize it value."""
        assert x0.size == self.nVars
        if method == 'least_square':
            xOpt = least_square(self.computeCostFunction, x0)
        elif method == 'curve_fit':
           v=1 
        elif method =='pso':
            v=2
        elif method == 'dea':
            v= 2
        else:
            logger.error('Unsupported Optimisation method !')
        
        return xOpt