import numpy as np

class BLDC:
    """
    BLDC - Brushless Direct Current Motor Model Function.

    Args:
        samplingRate (float): Sampling frequency of the input time-vectors.
        dQ_dt (numpy array): Motor rotor velocity.
        Q_t (numpy array): Motor rotor position.

    Keyword Args:
        Jm (float): Robot inertia factor.
        kf (float): Motor damping coefficient.
        Kt (float): Motor current coefficient.
        Tck (float): Motor cogging torque coefficients.
        Ta (float): Motor mechanical disturbance coefficients.
        Tb (float): Motor mechanical disturbance coefficients.

    Returns:
        tuple: Armature current vector (Ia), 
        Motor developed torque vector (Td).

    Ref:
        Practical Modeling and Comprehensive System Identification of a BLDC 
        Motor - C.Jiang, X.Wang, Y.Ma, B.Xu - 2015.
    """
    def __init__(self, Q_t, dQ_dt, d2Q_d2t, inertia=0.000558, torqueConstant=0.11, \
        damping=0.14, Tck=None, Ta=0.22, Tb=0.21):
        if Tck is None:
            Tck = [0.02, 0.02, 0.02, 0.02, 0.02]
        self.Q_t = Q_t
        self.dQ_dt = dQ_dt
        self.d2Q_d2t = d2Q_d2t
        self.J = inertia
        self.Kt = torqueConstant
        self.Kf = damping
        self.Tck = Tck
        self.Ta = Ta
        self.Tb = Tb
    
    def getTorque(self):
        Td = self.J * self.d2Q_d2t + self.Kf * self.dQ_dt - self.Ta * np.sin(self.Q_t) - self.Tb * np.cos(self.Q_t)
        for j in range(1, len(self.Tck) + 1):
            Td += self.Tck[j - 1] * np.cos(j * self.Q_t)
        return Td
    
    def getArmatureCurrent(self):
        Td = self.J * self.d2Q_d2t + self.Kf * self.dQ_dt - self.Ta * np.sin(self.Q_t) - self.Tb * np.cos(self.Q_t)
        for j in range(1, len(self.Tck) + 1):
            Td += self.Tck[j - 1] * np.cos(j * self.Q_t)
        I = Td / self.Kt
        return I 
        