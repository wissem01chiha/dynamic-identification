import logging
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import place_poles

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Kalman:
    """
    Classical Kalman filter identification algorithms base class.
    
    Args:
    
        F (np.ndarray): State transition model.
        H (np.ndarray): Observation model.
        Q (np.ndarray): Covariance of the process noise.
        R (np.ndarray): Covariance of the observation noise.
        P (np.ndarray): Initial error covariance.
        x0 (np.ndarray): Initial state estimate.
    Ref:
        An Introduction to the Kalman Filter - Greg Welch and Gary Bishop.
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
        innovations = []
        for z in observations:
            self.predict()
            self.update(z)
            y = self.update(z)
            states.append(self.x)
            innovations.append(y)
        return np.array(states), np.array(innovations)

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
        
    def visualizeEstimates(self, title=None, ylabel =None):
        """Visualize the estimated states over time."""
        plt.figure(figsize=(12, 6))
        for i in range(self.x.shape[1]):
            plt.plot(self.x[:, i], label=f'Estimated state {i+1}')
        plt.xlabel('Time (ms)')
        if not(title is None):
            plt.title(title,fontsize=9)
        if not(ylabel is None):
            plt.ylabel(ylabel,fontsize=9)
        plt.legend()


class RobustKalman:
    """
    Robust Kalman Filter Base Model Class 
    """
    def __init__(self, A, B, H, Q, R, P0, x0):
        self.A = A  # State transition matrix (time-varying)
        self.B = B  # Control input matrix (time-varying)
        self.H = H  # Observation matrix (time-varying)
        self.Q = Q  # Process noise covariance matrix (time-varying)
        self.R = R  # Measurement noise covariance matrix (time-varying)
        self.P = P0  # Initial covariance estimate
        self.x = x0  # Initial state estimate
        self.n = A.shape[0]

    def predict(self, u, k):
        """Prediction step"""
        A_k = self.A[:, (k-1)*self.n:k*self.n]
        B_k = self.B[:, (k-1)*self.n//2:k*self.n//2]
        Q_k = self.Q[:, (k-1)*self.n:k*self.n]
        np.random.seed(42)
        A_k = self._stabilize(A_k,B_k,-np.abs(np.random.rand(14)))
        self.x = A_k @ self.x + B_k @ u
        self.P[:, (k-1)*self.n:k*self.n] = A_k @ self.P[:, (k-1)*self.n:k*self.n] @ A_k.T + Q_k

        if np.isnan(self.x).any() or np.isnan(self.P).any():
            raise ValueError("NaN encountered in predict step")

    def update(self, z, k):
        """Update step"""
        H_k = self.H[:,(k-1)*self.n:k*self.n]
        R_k = np.diag(self.R[:, k])
        y = z - H_k @ self.x
        S = H_k @ self.P[:, (k-1)*self.n:k*self.n] @ H_k.T + R_k
        K = self.P[:, (k-1)*self.n:k*self.n] @ H_k.T @ np.linalg.inv(S)
        self.x  = self.x + K @ y
        I = np.eye(H_k.shape[1])
        self.P[:, (k-1)*self.n:k*self.n] = (I - K @ H_k) @ self.P[:, (k-1)*self.n:k*self.n]

        if np.isnan(self.x).any() or np.isnan(self.P).any():
            raise ValueError("NaN encountered in update step")

    def step(self, u, z, k):
        """One step of prediction and update"""
        self.predict(u, k)
        self.update(z, k)
        return self.x, self.P
    
    def _stabilize(self,A,B,desired_poles:np.ndarray):
        """ 
        Check what ever the numerical recursive control scheme given by :
                  x(k+1) = Ax(k) + B u
        is stable or not and adjust  it if necessary within ploes placement 
        strategy.
        """
        rank_B = np.linalg.matrix_rank(B)
        if rank_B == 0:
            logger.error("The control matrix B has rank 0")
        desired_poles = desired_poles.tolist()
        pole_counts = {pole: desired_poles.count(pole) for pole in set(desired_poles)}
        for pole, count in pole_counts.items():
            if count > rank_B:
                logger.error(\
            f"The pole {pole} is repeated {count} times, more than the rank of B ({rank_B}).")
        eigenvalues = np.linalg.eigvals(A)
        if np.any(np.abs(eigenvalues) < 1) :
            result = place_poles(A, B, desired_poles)
            k = result.gain_matrix 
            A_new = A - np.dot(B, k)
        else:
            A_new = A
        return A_new