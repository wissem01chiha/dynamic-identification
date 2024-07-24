from abc import ABC, abstractmethod

class TrajectoryOptimizer(ABC):
    
    @abstractmethod
    def computeTrajectoryCost():
        """ """
        pass
    
    @abstractmethod
    def computeTrajectoryMinParams(self):
        """ """
        pass
    
    @abstractmethod 
    def computetrajectoryGradient(self):
        pass 
    
    @abstractmethod 
    def computeTrajectoryBounds(self):
        pass 