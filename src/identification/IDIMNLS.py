import logging
import numpy as np
from typing import Callable
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd 
from scipy.optimize import least_squares, curve_fit 
from utils import RMSE

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
    def __init__(self,nVars,output,identificationModel: Callable[[np.ndarray],np.ndarray],time_step=0.001) -> None:
        self.time_step        = time_step
        self.output           = output
        self.nVars            = nVars
         
        self.identificationModel  = identificationModel
        self.optimized_params = None
        assert (np.ndim(self.output) == 2)
       
    def __str__(self) -> str:
        return (f"IDIMNLS Model with {self.nVars} optimization variables,"
                f"output shape: {self.torque.shape}, "
                f"time step: {self.time_step}")
        
    def computeLsCostFunction(self, x:np.ndarray):
        """
        The object function to be minimized with least squares.
        Returns:
            - cost : (float)
        """
        if self.nVars < x.size :
            xnew = np.concatenate([x[0:self.nVars], np.zeros(x.size-self.nVars)])
            tau_s = self.identificationModel(xnew)
        elif self.nVars == x.size:
            tau_s = self.identificationModel(x)
        else:
            logger.error(\
        'Identification Engine: Optimisation Variables number should be <= optimisation vector. ')
        
        rmse = RMSE(self.output, tau_s) 
        cost = np.mean(rmse**2)
        return cost 

    def evaluate(self)->None:
        """Evaluate the model's performance using the current parameters."""
        
        if self.optimized_params is None:
            logger.error("No optimized parameters found. Run optimize() first.")
            return
        tau_s = self.identificationModel(self.optimized_params)
        rmse = RMSE(self.output, tau_s)
        mean_rmse = np.mean(rmse)
        logger.info(f"Evaluation result - Mean RMSE: {mean_rmse:.4f}")
    
    def optimize(self, x0:np.ndarray, method='least_square', tol=0.5):
        """
        Optimize the cost function with NLS alorithms to mimize it value.
        Args:
            x0 : numpy-ndarry : initial paramters values estimation.
            method : optimisation algorithm
            tol : optimization alorithm error stop tolerence.
        """
        assert x0.size == self.nVars
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
                self.optimized_params = popt
            except Exception as e:
                logger.error(f"An error occurred during optimization: {e}")
        else:
            logger.error('Optimisation method Not Supported!')
        return xOpt
    
    def visualizeCostFunction(self)->None:
        """ """
    
    
    
    
    
    def visualizeError(self,title=None, ylabel=None)->None:
        """Plot the error between simulated and real torque using Seaborn."""
        
        if self.optimized_params is None:
            logger.error("No optimized parameters found. Run optimize() first.")
            return
        tau_s = self.identificationModel(self.optimized_params)
        error = self.torque - tau_s
        time_steps = np.arange(error.shape[0])
        error_df = pd.DataFrame(error, columns=[f'Joint {i+1}' for i in \
            range(error.shape[1])])
        error_df['Time Step'] = time_steps
        error_df_melted = error_df.melt(id_vars='Time Step', \
            var_name='Joint', value_name='Error')

        plt.figure(figsize=(12, 6))
        sns.lineplot(data=error_df_melted, x='Time Step', y='Error', hue='Joint')
        if not(title is None):
            plt.title(title,fontsize=9)
        plt.xlabel('Time (ms)',fontsize=9)
        if not(ylabel is None):
            plt.ylabel(ylabel,fontsize=9)
        plt.legend(title='Joint',fontsize=9)
        
def visualizeResults(self, title=None, y_label=None)->None:
    """Plot the simulated and real torque in one figure."""
    
    if self.optimized_params is None:
        logger.error("No optimized parameters found. Run optimize() first.")

    tau_s = self.identificationModel(self.optimized_params)
    time_steps = np.arange(self.torque.shape[0])

    real_df = pd.DataFrame(self.torque, columns=[f'Joint {i+1}' for i in \
        range(self.torque.shape[1])])
    simulated_df = pd.DataFrame(tau_s, columns=[f'Joint {i+1}' for i in \
        range(tau_s.shape[1])])

    real_df['Time'] = time_steps
    simulated_df['Time'] = time_steps

    combined_df = pd.merge(real_df.melt(id_vars='Time', var_name='Joint', \
        value_name='Real Torque'),
                           simulated_df.melt(id_vars='Time', var_name='Joint',\
                               value_name='Simulated Torque'),
                           on=['Time', 'Joint'])

    plt.figure(figsize=(12, 6))
    sns.lineplot(data=combined_df, x='Time', y='Real Torque', hue='Joint',\
        style='Joint', dashes=False, markers=True, alpha=0.6)
    sns.lineplot(data=combined_df, x='Time', y='Simulated Torque', hue='Joint',\
        style='Joint', dashes=True)

    if title is not None:
        plt.title(title)
    plt.xlabel('Time (ms)', fontsize=9)
    if y_label is not None:
        plt.ylabel(y_label, fontsize=9)
    plt.legend(title='Joint', fontsize=9)