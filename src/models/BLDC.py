import numpy as np

class BLDC:
    """
    BLDC - Brushless Direct Current Motor Model Function.

    Args:
        dQ_dt (numpy array): Motor rotor velocity.
        d2Q_d2t (numpy array) : Motor rotor acceleration.
        Q_t (numpy array): Motor rotor position.

    Keyword Args:
        Jm (float): Robot inertia factor.
        kf (float): Motor damping coefficient.
        Kt (float): Motor current coefficient.
        Tck (float): Motor cogging torque coefficients.
        Ta (float): Motor mechanical disturbance coefficients.
        Tb (float): Motor mechanical disturbance coefficients.

    Returns:
        - Ia Armature current vector.
        - Td : Motor developed torque vector.

    Ref:
        Practical Modeling and Comprehensive System Identification of a BLDC 
        Motor - C.Jiang, X.Wang, Y.Ma, B.Xu - 2015.
    """
    def __init__(self,  inertia=0.558, torqueConstant=0.11, \
        damping=0.14, Tck=None, Ta=0.22, Tb=0.21,Imax=5,Tmax=25):
      
        if Tck is None:
            Tck = [0.015, 0.018, 0.023, 0.0201, 0.0147]
        self.J = inertia
        self.Kt = torqueConstant
        self.Kf = damping
        self.Tck = Tck
        self.Ta = Ta
        self.Tb = Tb
        self.Imax = Imax
        self.Tmax = Tmax
    
    def computeOutputTorque(self, Q_t, dQ_dt, d2Q_d2t):
        Td = self.J * d2Q_d2t + self.Kf * dQ_dt - self.Ta * np.sin(Q_t) - self.Tb * np.cos(Q_t)
        for j in range(1, len(self.Tck) + 1):
            Td += self.Tck[j - 1] * np.cos(j * Q_t)
        return Td
    
    def computeArmatureCurrent(self,Q_t, dQ_dt, d2Q_d2t):
        Td = self.J * d2Q_d2t + self.Kf * dQ_dt - self.Ta * np.sin(Q_t) - self.Tb * np.cos(Q_t)
        for j in range(1, len(self.Tck) + 1):
            Td += self.Tck[j - 1] * np.cos(j * Q_t)
        Td = self.checkTorque(Td)
        I = Td / self.Kt
        I = self.checkCurrent(I)
        return I
    
    def checkCurrent(self, I):
        """ Check the computed actuator current, clip it if exceeded """
        I = np.clip(I, -self.Imax, self.Imax)
        return I
    
    def checkTorque(self, T):
        """ Check the computed actuator torque, clip it if exceeded """
        T = np.clip(T, -self.Tmax, self.Tmax)
        return T