import numpy as np 
from dynamics import Robot

class StateSpace:
    """
    Base class for state space identification models 
    
    Args:
        robot  - 
    """
    def __init__(self,robot:Robot) -> None:
        self.robot = robot
        pass
    
    def computeStateMatrices(self):
        """Compute the state space matrices of the robot model """
        A = np.zeros((2*self.robot.nq,2*self.robot.nq))
        B = 1
        return A, B, C, D 
    
    def getStateVector(self):
        """ Compute the State vector """
        assert self.robot.model.nq == self.robot.model.nv,"config vector size and velocity must be equal"
        
        x = 1
        return x
    
    def computeStatevector(self):
        """ Compute the disrceate state vector at time date t+1"""
        x_dot = 1
        return x_dot
        
    def obs(self):
        """compute the  observaliblite matrix """
        obs_matrix=1
        return obs_matrix
    
    def ctlb(self):
        """ compute the controllabilty matrix of the robot"""
        ctlb_matrix=1
        return ctlb_matrix
    
    def getAugmentedStateVector(self):
        """ Compute the Augemnted Satte space model"""
        return 1