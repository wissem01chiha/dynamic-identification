import numpy as np
from utils import columnVector
class Dahl:
    """
    Dahl friction Model class base definition.
    The friction force is a hysteresis function, without memory of the x
    Args:
        - sigma0: Constant coefficient
        - Fs    : Stribeck force coefficient
    """
    def __init__(self, sigma0, Fs, time_step=0.001) -> None:
        assert sigma0 is not None and sigma0 != 0, \
            "Viscoelastic Engine: coefficient sigma must be non-null float."
        assert Fs is not None and Fs != 0, \
            "Viscoelastic Engine: coefficient Fs must be non-null float."
        self.sigma0    = sigma0 
        self.Fs        = Fs     
        self.time_step = time_step 

    def computeFrictionForce(self, velocity:np.ndarray) -> np.ndarray:
        """
        Compute the friction force based on the Dahl model.
        
        Args:
            - velocity (np.ndarray): velocity values.
            
        Returns:
            np.ndarray: computed friction forces.
        """
        time_span = (velocity.size-1)* self.time_step
        t = np.linspace(0, time_span, velocity.size)
        F = np.zeros_like(velocity)
        dFdt = np.zeros_like(F)
        for i in range(1,velocity.size):
            dFdt[i] = self.dahl(F[i-1],velocity[i])
            F[i] = dFdt[i] *self.time_step + F[i-1]
        return F
    
    def dahl(self,F,v):
        if v == 0:
            dFdt = 0
        else:
            dFdt = self.sigma0 /v*(1- F/self.Fs*np.sign(v))
        return dFdt