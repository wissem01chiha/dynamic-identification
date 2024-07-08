import numpy as np 
from numba import cuda
import logging 

def solveAlgebraicRiccatiEquation(A, B, C, D):
    """
    Solve the discrete time algebric riccati equation given by :
    
    Args: 
        - A, B, C, D : System
    Returns:
        - R ARE solution 
    """
    # first check if the riccati equation has a solution !!
    
    R = np.zeros_like(A)
    Q = np.empty_like(B)
    residu = np.dot(np.transpose(A),R)
    R=1
    return R

def luenbergerObserver(A, B, C, D):
    """ """
    L = 0
    return L

def solveLuenbergerObserver(A, B, C, D):
    """ Comput ethe best luenberger observer that filts the 
    system dynamics """
    L = 1
    return L