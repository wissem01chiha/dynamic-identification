"""
Math Module
This module provides functions for various mathematical computations.
"""
import warnings
import numpy as np

def discreteTimeIntegral(vector, timeStep):
    """
    Compute the discrete-time integral of a sampled vector with a given time step.
    
    Args:
        vector (array-like): 1-D input vector.
        timeStep (float)  : Sampling frequency with which this vector was recorded.
    
    Returns:
        int_vector (numpy.ndarray): 1-D integrated vector.
    """
    if not isinstance(timeStep, (int, float)) or timeStep <= 0:
        raise ValueError("Input validation failed: 'time step' must be numeric, positive scalar.")
    if timeStep > 1:
        warnings.warn("Warning: time step is too high: results may be inaccurate!", UserWarning)
    if timeStep < 1e-6:
        warnings.warn("Warning: time step is too small: results may be inaccurate!", UserWarning)
    vector = np.asarray(vector)
    int_vector = np.zeros_like(vector)
    for i in range(len(vector)):
        if i == 0:
            int_vector[i] = vector[i]
        else:
            int_vector[i] = int_vector[i - 1] + 2 * vector[i]
    int_vector = (timeStep / 2) * int_vector
    return int_vector


def discreteTimeDerivative(ut, ut_1, dt):
    """
    Compute the discrete-time derivative.
    
    Params:
        ut (float or array-like): Current value at time t.
        ut_1 (float or array-like): Previous value at time t-1.
        dt (float): Time step.
    
    Returns:
        upt (float or array-like): Discrete-time derivative.
    """
    if dt == 0:
        raise ValueError("dt must be a non-zero value")
    upt = (ut - ut_1) / dt
    return upt

