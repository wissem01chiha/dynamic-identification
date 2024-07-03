import logging 
from typing import overload
import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def conditionNumber(M, threshold=1e-5):
    """
    Computes the condition number of a matrix with a check for SVD convergence.
    Args:
        - M         : The input matrix.
        - threshold : The condition number threshold to check against.
    """
    try:
        cond_number = np.linalg.cond(M)
        return cond_number < threshold
    except np.linalg.LinAlgError as e:
        logger.error(f"Linear Algebra error: {e}")
        return False

def discreteTimeIntegral(vector, time_step):
    """
    Compute the discrete-time integral of a sampled vector with a given time step.
    
    Args:
        vector (array-like): 1-D input vector.
        time_step (float)  : Sampling frequency with which this vector was recorded.
    
    Returns:
        int_vector (numpy.ndarray): 1-D integrated vector.
    """
    if not isinstance(time_step, (int, float)) or time_step <= 0:
       logger.error("Input validation failed: 'time step' must be numeric, positive scalar.")
    if time_step > 1:
        logger.warning("Warning: time step is too high: results may be inaccurate!")
    if time_step < 1e-6:
        logger.warning("Warning: time step is too small: results may be inaccurate!")
    vector = np.asarray(vector)
    int_vector = np.zeros_like(vector)
    for i in range(len(vector)):
        if i == 0:
            int_vector[i] = vector[i]
        else:
            int_vector[i] = int_vector[i - 1] + 2 * vector[i]
    int_vector = (time_step / 2) * int_vector
    return int_vector

@overload
def discreteTimeDerivative(ut, ut_1, dt):
    """
    Compute the discrete-time derivative in a single time step.
    
    Args:
        ut (float or array-like): Current value at time t.
        ut_1 (float or array-like): Previous value at time t-1.
        dt (float): Time step.
    
    Returns:
        upt (float or array-like): Discrete-time derivative.
    """
    if dt == 0:
        logger.error("dt must be a non-zero value")
    upt = (ut - ut_1) / dt
    return upt

@overload
def discreteTimeDerivative(vector:np.ndarray, time_step:float, init_value=0):
    """
    Compute the discrete-time derivative.
    """
    if time_step ==0:
        logger.error("time_step must be a non-zero value")
    der_vector = np.zeros_like(vector)
    der_vector[0] = init_value
    for i in range(1,len(vector)):
        der_vector[i] = discreteTimeDerivative(vector[i],vector[i-1],time_step)
    return der_vector

def RMSE(array1: np.ndarray, array2: np.ndarray,axis=0) -> np.ndarray:
    """
    Compute the RMSE between 2 arrays across all samples.
    
    Args:
        array1  (Nsamples, ndof).
        array2  (Nsamples, ndof).
        
    Returns:
        np.ndarray: (Nsamples, ndof)
    """
    if array1.shape != array2.shape:
        logger.error("Input arrays must have the same shape.")
    differences = array1 - array2
    squared_diff = differences ** 2        
    mean_squared_diff = np.mean(squared_diff, axis)
    rmse = np.sqrt(mean_squared_diff)
    return rmse

def cumulativeRMSE(array1: np.ndarray, array2: np.ndarray) -> np.ndarray:
    """
    Compute the RMSE between columns of two arrays, considering 
    the error committed for each joint at the previous time step.
    
    Args:
        array1 (Nsamples, ndof).
        array2 (Nsamples, ndof).
        
    Returns:
        np.ndarray: (Nsamples, ndof)
    """
    differences = array1 - array2
    squared_diff = differences ** 2
    cumulative_error = np.cumsum(squared_diff, axis=0)
    mean_squared_diff = cumulative_error / (np.arange(1, array1.shape[0] + 1)[:, np.newaxis])
    rmse = np.sqrt(mean_squared_diff)
    return rmse

def computeCorrelation(array: np.ndarray) -> np.ndarray:
    """
    Compute the correlation factor between the columns of a numpy array.
    
    Args:
        array (Nsamples, ndof).
        
    Returns:
        np.ndarray: (ndof, ndof).
    """
    return np.corrcoef(array, rowvar=False)

def computeCumulativeCorrelation(array:np.ndarray)->np.ndarray:
    """
    Compute the correlation matrix.
    Args:
        - array : (Nsamples, ndof).
    Returns:
        - numpy-ndarry: (Nsamples,ndof, ndof)
    """
    rows, cols = array.shape
    correlation_matrix = np.zeros(rows, cols, cols)
    for i in range(1, rows):
        correlation_matrix[i,:,:]= np.corrcoef(array[0:i,:], rowvar=False)
    return correlation_matrix

def computeMovingAverage(array: np.ndarray, window_size: int) -> np.ndarray:
    """
    Implement a simple moving average estimator.
    
    Args:
        array (np.ndarray): Array of values at each timestep.
        window_size (int): Number of previous values to consider.
        
    Returns:
        np.ndarray: expected values.
    """
    expected_values = np.zeros_like(array)
    for i in range(len(array)):
        if i < window_size:
            expected_values[i] = np.mean(array[:i+1])
        else:
            expected_values[i] = np.mean(array[i-window_size+1:i+1])
    return expected_values