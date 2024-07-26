import numpy as np 
import matplotlib.pyplot as plt 
import seaborn as sns 
import logging 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from utils import plotArray

class SplineGenerator:
  
    def __init__(self, trajectory_params) -> None:
        self.trajector_params = trajectory_params
        
    
    def computeTrajectoryState(self,t,Q0=None)->np.ndarray:
        """
        > Computes the trajectory states at time date t 
        """
        states = 1
        return states
    
    def computeTrajectoryIdentifiability(self):
        """ 
        evaluates the rgression criteria ε(q,qp,qpp,x) , en fonction du trajectory certion computed 
        pervoiously C(q,qp,qpp) for fixed x system paramter vector :
        > if we minimize 
        """

        
    def computeFullTrajectory(self,ti:float,tf:float,q0=None,qp0=None,qpp0=None):
        """ """
        return  q, qp ,qpp
    
    def computeTrajectoryCriterion(self, t_i:float, t_f:float)->float:
        return 
    
    def computeTrajectoryConstraints(self,qmax,qmin,qpmax,qpmin,qppmin,qppmax,\
        ti,tf,q0=None, qp0= None, qpp0=None):
        """ """
        return 
    
    def visualizeTrajectory(self, ti,tf, Q0=None, Qp0=None, Qpp0=None):
        
        q, qp, qpp = self.computeFullTrajectory(ti,tf,Q0,Qp0,Qpp0)
        plotArray(q,'Computed Trajectory Joints Positions')
        plotArray(qpp,'Computed Trajectory Joints Accelerations')
        plotArray(qp,'Computed Trajectory Joints Velocity')
        
    def saveTrajectory2file(self):
        # for send to balst kinova 
        """ """