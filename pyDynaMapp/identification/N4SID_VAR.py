import numpy as np 
import matplotlib.pyplot as plt 
from ..dynamics import StateSpace 

class N4SID:
    """
     Numerical Subspace State Space System Identification
     This method uses input-output data to construct a state-space model by minimizing 
     a prediction error criterion.
     
     
    Ref: 
        "N4SID: Subspace algorithms for the identification of combined deterministic-stochastic systems."
        Van Overschee, Peter, and Bart De Moor - Automatica 30.1 (1994): 75-93
    """
    def __init__(self) -> None:
        
        pass