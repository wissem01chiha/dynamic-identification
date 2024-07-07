import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns 

class Kalman:
    """
    KALMAN
    
    Classical Kalman filter identification algorithms base class.
    
    Args:
        F (np.ndarray): State transition model.
        H (np.ndarray): Observation model.
        Q (np.ndarray): Covariance of the process noise.
        R (np.ndarray): Covariance of the observation noise.
        P (np.ndarray): Initial error covariance.
        x0 (np.ndarray): Initial state estimate.
    """
    
    def __init__(self, F, H, Q, R, P, x0, alpha=0.2, beta=1):
        self.F = F   
        self.H = H   
        self.Q = Q   
        self.R = R   
        self.P = P   
        self.x = x0 
        self.alpha = alpha
        self.beta = beta

    def predict(self):
        """Predict the state and error covariance for the next time step."""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        """
        Update the state estimate and error covariance using the observation.
        
        Args:
            z (np.ndarray): Observation at the current time step.
        """
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)  
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        self.adaptNoiseCovariance(y)

    def filter(self, observations):
        """
        Apply the Kalman filter to a sequence of observations.
        
        Args:
            observations (np.ndarray): Sequence of observations.
        
        Returns:
            states (np.ndarray): Sequence of state estimates.
        """
        states = []
        for z in observations:
            self.predict()
            self.update(z)
            states.append(self.x.copy())
        return np.array(states)

    def evaluate(self, true_states, estimated_states):
        """
        Evaluate the performance of the Kalman filter.
        
        Args:
            true_states (np.ndarray): True states.
            estimated_states (np.ndarray): Estimated states from the filter.
        
        Returns:
            performance (dict): Dictionary containing evaluation metrics.
        """
        mse = np.mean((true_states - estimated_states) ** 2)
        performance = {'MSE': mse}
        return performance
    
    def adaptNoiseCovariance(self, y):
        """
        Adaptively update the process and observation noise covariances.
        
        Args:
            y (np.ndarray): Innovation or measurement residual.
        """
        self.Q = self.Q + self.alpha * (np.outer(y, y) - self.Q)
        self.R = self.R + self.beta * (np.outer(y, y) - self.R)
        
    def visualizeSatets(self):
        """ """
        plt.figure(figsize=(12, 6))