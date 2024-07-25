from abc import ABC, abstractmethod
from typing import Callable
import numpy as np


class IDIM:
    """ Inertface class for Inverse Dynamics identification models """
    @abstractmethod
    def set_upper_bounds(self):
        pass
    
    @abstractmethod
    def set_lower_bounds(self):
        pass

    @abstractmethod
    def set_maxeval(self):
        pass

    @abstractmethod
    def set_set_min_objective(self,f:Callable[[np.ndarray],np.ndarray]):
        pass

    @abstractmethod
    def optimize(self,x0:np.ndarray):
        pass

    @abstractmethod 
    def opt(self,method:str):
        pass
    
    @abstractmethod
    def vizulizeOuput(self):
        pass

    @abstractmethod
    def save(self,folder_path:str):
        pass

class IDIMMLE(IDIM):
    """
    Inverse dynamics identification with maximum likelihood estimation.
    Ref:
        Fourier-based optimal excitation trajectories for the dynamic identification of robots
        Kyung.Jo Park - Robotica - 2006.
    """
    def __init__(self) -> None:
        pass
    def set_lower_bounds(self):
        return super().set_lower_bounds()
    
    def set_maxeval(self):
        return super().set_maxeval()
    
    def set_set_min_objective(self, f: Callable[[np.ndarray], np.ndarray]):
        return super().set_set_min_objective(f)

