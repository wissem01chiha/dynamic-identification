import numpy as np 
from scipy.linalg import solve_discrete_are
import logging 

def solveAlgebraicRiccatiEquation(A, B, Q, R):
    """
    Solve the discrete time algebric riccati equation given by :
    
    Args: 
        - A, B, C, D : System
    Returns:
        - R ARE solution. 
    """
    assert A.shape[0] == A.shape[1], "Matrix A must be square."
    assert B.shape[0] == A.shape[0], "Matrix B must have the same number of rows as A."
    assert Q.shape[0] == Q.shape[1] == A.shape[0], "Matrix Q must be square and match the dimensions of A."
    assert R.shape[0] == R.shape[1] == B.shape[1], "Matrix R must be square and match the number of columns of B."
    
    R  =  solve_discrete_are(A, B, Q, R)
    return R

def solve_discrete_state_depend_are(A, B, Q, R):
    """ 
    Solve the discrete state depend Riccati equation 
    Ref:
    
    """

def luenbergerObserver(A, B, C, D):
    """ """
    L = 0
    return L

def solveLuenbergerObserver(A, B, C, D):
    """ Comput ethe best luenberger observer that filts the 
    system dynamics """
    L = 1
    return L