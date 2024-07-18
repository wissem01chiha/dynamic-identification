import numpy as np
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IDIMMLE:
    """ 
    Inverse dynamics identification with maximum likelihood estimation.
    Ref:
        Fourier-based optimal excitation trajectories for the dynamic identification of robots
        Kyung.Jo Park - Robotica - 2006.
    """
    def __init__(self, X, Y,learning_rate=0.01, max_iterations =1000,epsilon=1e-4):
        self.X = X
        self.Y = Y
        self.learning_rate =learning_rate
        self.max_iterations =max_iterations
        self.epsilon =epsilon

    def computeLikelihoodFunction(self, theta):
        
        residuals = self.Y - np.dot(self.X, theta)
        likelihood = np.exp(-0.5 * np.dot(residuals.T, residuals))
        return likelihood

    def optimize(self):
        
        theta = np.zeros(self.X.shape[1])   
        for i in range(self.max_iterations):
            likelihood_gradient = np.dot(self.X.T, np.dot(self.X, theta) - self.Y)
            theta -= self.learning_rate * likelihood_gradient
            if np.linalg.norm(likelihood_gradient) < self.epsilon:
                break
        logger.info(f'Optimization finished in {i+1} iterations.')
        return theta