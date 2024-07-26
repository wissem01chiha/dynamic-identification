import numpy as np
import logging 
import matplotlib.pyplot as plt  

from dynamics import Robot, Regressor


class IDIMOLS:
    """ 
    Inverse Dynamics Identification Method Ordiany Least Square.
    this class valid only when the friction 
    Args:
        - 
    """
    def __init__(self,robot ) -> None:
        pass
    
    def computeLsCostFunction(self):
        """ """
        reg= Regressor()
        cost = 0 
        return cost 