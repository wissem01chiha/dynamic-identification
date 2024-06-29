import logging
import numpy as np 
from typing import overload 

from dynamics import Robot

class StateSpace:
    """
    Base class for state space identification models 
    
    Args:
        robot  - Manipulator Base model 
    """
    def __init__(self,robot:Robot=None) -> None:
        if robot is None:
            self.robot = Robot()
            
    @overload
    def computeStateMatrices(self,q:np.ndarray,qp:np.ndarray):
        """
        Compute the state space matrices of the robot model.
        Args:
            - qp : Joints velocity vector 
            - q : Joints position vector 
        """
        assert q.size == qp.size
        n = self.robot.model.nq
        A = np.zeros((2*n,2*n))
        B = np.zeros((2*n,n))
        C = np.zeros((n,2*n))
        D = np.zeros((n,n))
        C = self.robot.computeCorlolisMatrix(qp,q)
        G = self.robot.computeGravityTorques(q)
        M = self.robot.computeMassMatrix(q)
        K = self.robot.getStiffnessMatrix()
        A[n:2*n,n:2*n]= -np.linalg.inv(M)*C
        A[0:n,n:2*n] = np.eye(n)
        A[n:2*n,0:n]= -np.linalg.inv(M)*K
        A[0:n,0:n]= np.zeros(n)
        C[0:n,0:n]= np.eye(n)
        return A, B, C, D 
    
    @overload
    def computeStateMatrices(self, x:np.ndarray):
        """
        Compute the state space matrices of the robot model given the state vector.
        Args:
            - x: system state vector 
        """
        n = self.robot.model.nq
        assert x.size == 2*n, f"state vector size should be 2 * n ({2*n}), but got ({x.size}) instead"
        q =  x[0:n]
        qp = x[n:2*n]
        A, B, C, D = self.computeStateMatrices(q, qp)
        return A, B, C, D 
    
    
    def getStateVector(self,qp:np.ndarray,q:np.ndarray):
        """ Compute the State vector."""
        assert np.size(qp) == np.size(q),\
            "position and velocity vector sizes must be equal"
        x = np.concatenate(q, qp, axis=0)
        return x
    
    def updateStateVector(self, q:np.ndarray, qp:np.ndarray, tau:np.ndarray):
        """ 
        Compute the disrceate state vector at time date t + 1 give position 
        and velocity and torques vetcor at time date t. 
        """
        assert q.size == qp.size, "Input position and velocity vectors mush have same size"
        A, B, _, _ = self.computeStateMatrices(q, qp)
        x_k = self.getStateVector(qp,q)
        x_k_1 = A * x_k + B * tau
        return x_k_1
        
    def computeStateInputVector(self, q:np.ndarray, qp:np.ndarray, qpp:np.ndarray, \
        tau:np.ndarray=None, noise:bool=False):
        """ """
        n = self.robot.model.nq
        tau_g = self.robot.computeGravityTorques(q)
        tau_f = self.robot.computeFrictionTorques(qp)
        tau_m = self.robot.computeActuatorTorques(q,qp,qpp)
        if tau is None:
            tau = tau_m
        u = tau_f + tau_g - tau
        # adress the problem where some values in u are grater than others wich can lead to 
        # numercial issues 
        return u    
        
    def computeObsMatrix(self, qp: np.ndarray, q:np.ndarray):
        """Compute the observaliblite matrix of the robot"""
        n = self.robot.model.nq
        A, _, C, _ = self.computeStateMatrices(q, qp)
        obs_matrix = C
        for i in range(1, 2*n):
            obs_matrix = np.vstack((obs_matrix, C @ np.linalg.matrix_power(A, i)))        
        return obs_matrix
    
    def computeCtlbMatrix(self, qp:np.ndarray, q:np.ndarray):
        """ Compute the controllabilty matrix of the robot"""
        n = self.robot.model.nq
        ctlb_matrix = B
        A, B, _, _ = self.computeStateMatrices(q,qp)
        return ctlb_matrix
    
    def getAugmentedStateVector(self, q:np.ndarray, qp :np.ndarray, tau:np.ndarray):
        """ Compute the Augemnted state vector"""
        x= self.getStateVector(qp,q)
        z = np.concatenate(x,tau,axis=0)
        return z
    
    def computeAugemntedStateMatrices(self, q:np.ndarray, qp:np.ndarray):
        """ Computes and retuns """
        A, B, C, D = self.computeStateMatrices(qp,q)
        
    
    def getStateEigvals(self, q:np.ndarray, qp:np.ndarray):
        """ Returns the system eigvalues"""
        A, _, _, _ = self.computeStateMatrices(q,qp)
        eigvals = np.linalg.eigvals(A)
        return eigvals 
        
    def computeReducedStateMatrices(self, q:np.ndarray, qp:np.ndarray, tau:np.ndarray):
        """ 
        computes the new transformed system by diagonalized the state matrix
        A 
        """ 
        A, B, C, D = self.computeStateMatrices(q,qp)
        x = self.getStateVector(qp,q)
        xdot =self.updateStateVector(q,qp,tau)
        A_hat = np.eye(1)
        xdot_ht = 1
        x_hat =1
        B_hat = 1
        C_hat = 1
        D_hat =1    
    
    
    def simulate(self, x0:np.ndarray, input:np.ndarray=None, noise=None):
        """ 
        Simulate the system response with a given input torque.
        Args:
            - x0 : initial system state. ( 2.ndof * 1 )
            - input : Input torque to the system (NSamples  * ndof )
        Returns:
            - states : numpy-ndarry stroing iteration states vectors. ( 2.ndof * NSamples)
        """
        NSamples, ndof = input.shape
        n = self.robot.model.nq
        assert ndof == n, "ndof msitamtech"
        states = np.zeros(2*n, NSamples)
        states[:,0] = x0
        for i in range(1,NSamples):
            states[:,i] = self.updateStateVector(states[:,i-1],input[i,:])
        return states 
    
    def linearize(self,q, qp:np.ndarray, noise:bool=False):
        """ linearize by finite diffrence method the state-depend dynamics matrices """
        return 
    
    
    def computeGaussianNoise(self, stdv=1):
        """Returns a gaussian distribuation noise vector"""
        n = self.robot.model.nq
        noise_vector = np.random.normal(0, stdv, n)
        return noise_vector
    
    def computePoissonNoise(self, lam= 0.1):
        """ Returns a poisson distribuation noise vector"""
        n = self.robot.model.nq
        noise_vector = np.random.poisson(lam)
        return noise_vector  