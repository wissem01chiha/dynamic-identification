import numpy as np 
import matplotlib.pyplot as plt 
import  seaborn as sns 
import logging 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Spline:
  
    def __init__(self) -> None:
        pass
    
    def computeTrajectoryCriterion(self, t_i, t_f)->float:
        return 