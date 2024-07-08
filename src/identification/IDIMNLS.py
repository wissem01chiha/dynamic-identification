import logging
import numpy as np
from typing import Callable
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, curve_fit 
from utils import RMSE, clampArray, plotArray, plot2Arrays

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IDIMNLS:
    """
    Inverse Dynamics Identification Methods with Non Linear Least Square Alogrithms.
    The identification problem is formulated in a non linear optimisation problem :
    
        Xopt = argmin(X) ||IDM(q, qdot, qddot, X)- tau||
        
    Args:
        - nVars : number of optimization variables in the X vector.
        - output : desired output vector ( Nsamples * ndof )
        - identificationModel : function that return the model computed torques.
            of shape : ( Nsamples * ndof )
    """
    def __init__(self,nVars,output,identificationModel: Callable[[np.ndarray],np.ndarray],\
        upperBound=10,lowerBound=0.001,time_step=0.001) -> None:
        
        # Class Attributes
        self.time_step = time_step
        self.output = output
        self.nVars = nVars
        self.upperBound = upperBound
        self.lowerBound = lowerBound
        self.identificationModel  = identificationModel
        self.optimized_params = None
        assert (np.ndim(self.output) == 2)
       
    def __str__(self) -> str:
        return (f"IDIMNLS Model with {self.nVars} optimization variables,"
                f"output shape: {self.output.shape}, "
                f"time step: {self.time_step}")
        
    def computeLsCostFunction(self, x:np.ndarray):
        """
        The object function to be minimized with least squares.
        Returns:
            - cost : (float)
        """
        if self.nVars < x.size :
            xnew = np.concatenate([x[0:self.nVars], np.zeros(x.size-self.nVars)])
            xnew = clampArray(xnew,self.lowerBound,self.upperBound)
            tau_s = self.identificationModel(xnew)
        elif self.nVars == x.size:
            x = clampArray(x,self.lowerBound,self.upperBound)
            tau_s = self.identificationModel(x)
        else:
            logger.error(\
        'Identification Engine: Optimisation Variables number should be <= input vector size. ')
        
        rmse = RMSE(self.output, tau_s) 
        cost = np.mean(rmse**2)
        return cost 
    
    def computeRelativeError(self):
        relaErr =0
        return relaErr 
    
    
    def evaluate(self)->None:
        """Evaluate the model's performance using the current parameters."""
        
        if self.optimized_params is None:
            logger.error("No optimized parameters found. Run optimize() first.")
            return
        tau_s = self.identificationModel(self.optimized_params)
        rmse = RMSE(self.output, tau_s)
        mean_rmse = np.mean(rmse)
        logger.info(f"Evaluation result - Mean RMSE: {mean_rmse:.4f}")
    
    def optimize(self, x0:np.ndarray, method='least_square', tol=0.0001):
        """
        Optimize the cost function with NLS alorithms to mimize it value.
        Args:
            x0 : numpy-ndarry : initial paramters values estimation.
            method : optimisation algorithm
            tol : optimization alorithm error stop tolerence.
        """
        xOpt = x0
        if method == 'least_square':
            try:
                xOpt = least_squares(self.computeLsCostFunction, x0, xtol=tol)
                self.optimized_params = xOpt.x
            except Exception as e:
                logger.error(f"An error occurred during optimization: {e}")
        elif method == 'curve_fit':
            init_params = np.zeros(self.nVars)
            try:
                x_data = np.linspace(0, len(self.output) * self.time_step,\
                    len(self.output))
                popt, _ = curve_fit(self.identificationModel, x_data, \
                    self.output , p0=init_params,method='trf')
                self.optimized_params = popt.x
            except Exception as e:
                logger.error(f"An error occurred during optimization: {e}")
        else:
            logger.error('Optimisation method Not Supported!')
        return xOpt
    
    def visualizeCostFunction(self)->None:
        """
        Plot the cost function scalar variation respect to ||x|| only 
        the optimisation variables 
        are considerd 
        """
        plt.figure(figsize=(12, 6))
    
    
    def visualizeError(self,title=None, ylabel=None)->None:
        """Plot the root squred error between simulated and inputs"""
        
        if self.optimized_params is None:
            logger.error("Identification Engine : No optimized parameters found. Run optimize() first.")
            return
        tau_s = self.identificationModel(self.optimized_params)
        rmse = RMSE(self.output, tau_s,1)
        plotArray(rmse,'err per joint','abs err')
        
    def visualizeResults(self, title=None, y_label=None)->None:
        """Plot the simulated and real signals in one figure."""
    
        if self.optimized_params is None:
            logger.error("No optimized parameters found. Run optimize() first.")
        tau_s = self.identificationModel(self.optimized_params)
        plot2Arrays(tau_s,self.output,'simultion','true',title)
        
    def visualizeRelativeError(self):
        """ """
 

   