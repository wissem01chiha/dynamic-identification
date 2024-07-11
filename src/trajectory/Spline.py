import numpy as np 
import matplotlib.pyplot as plt 
import seaborn as sns 
import logging 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SplineGenerator:
  
    def __init__(self) -> None:
        pass
    
    def computeTrajectoryState(self,t,Q0=None)->np.ndarray:
        states =1
        return states
    
    def computeTrajectoryCriterion(self, t_i:float, t_f:float)->float:
        return 