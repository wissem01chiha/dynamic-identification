import numpy as np

class HarmonicDrive():
    """
    
    
    """
    def __init__(self,Tin,Vin,N = 100) -> None:
        self.reductionRatio = N
        self.inputTorque = Tin
        self.inputVelocity = Vin
        
    
    def getOutputVelocity(self):
        """ """
        V=1
        return V
    
    def getOutputTorque(self):
        """ """
        T=1
        return T