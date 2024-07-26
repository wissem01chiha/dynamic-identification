import numpy as np

def computeViscousFrictionForce(V, Fc, Fs):
    """
    Compute the Coulomb and viscous friction model.
    
    Parameters:
    V  (numpy array): Velocity array.
    Fc (float or numpy array): Coulomb friction coefficient.
    Fs (float or numpy array): Viscous friction coefficient.
    
    Returns:
        numpy-array: Friction force array.
    """
    return Fc * np.sign(V) + Fs * V

