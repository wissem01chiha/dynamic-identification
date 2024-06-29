import numpy as np 

class IDIMNLS:
    """
    Inverse Dynamics Identification Methods with Non Linear Least Square Alogrithms.
    the problem is formulated in a non linear optimisation problem :
    
        Xopt = argmin(X) || IDM(q, qdot, qddot)- tau||
    Args:
        nVars: number of optimization variables in the X vector.
        output : desired output vector (tau)
    """
    def __init__(self,nVars,output:np.ndarray, time_step=0.001) -> None:
        self.time_step  = 0.001
        self.output = output
       
    def computeCostFunction(self):
        """
        The object function to be minimized.
        
        Returns:
            cost : The computed cost.
        """
        cost = 0
        return cost 
        
    def updateParameters(self, params)->None:
        """ Updates system paramters
        Args:
            - params : 
        """
     
        
    def evaluate(self):
         """Evaluate the model's performance using the current parameters."""
    
    def optimize(self):
        """Optimize the cost function with NLS alorithms to mimize it value to 0  """
        fval = self.computeCostFunction() 
        Xopt = 0 
        return Xopt