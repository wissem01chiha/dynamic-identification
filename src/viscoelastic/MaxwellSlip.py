import numpy as np
from scipy.integrate import odeint

class MaxwellSlip:
    """
    MaxwellSlip - Compute Maxwell Slip Friction Model.

    Inputs:
      n           - Number of Maxwell elements.
      velocity    - Velocity (m/s)
      k           - Stiffness of Maxwell elements (N/m)
      c           - Damping coefficients of Maxwell elements (Ns/m)
      sigma0      - Static friction force (N)
      samplingRate- Sampling rate (Hz)

    Returns:
      t           - Simulation time vector.
      F           - Friction Force for the given velocity
    Note:
    
    Ref:
      Fundamentals Of Friction Modeling - Farid Al-Bender - 2010.
    """

    def __init__(self, n, velocity, k, c, sigma0, samplingRate=1000):
      assert samplingRate != 0,"Sampling frequency should not be null."
      self.n = n
      self.velocity = velocity
      self.k = k
      self.c = c
      assert len(self.k) == n,\
        "Length of stiffness coefficients (k) should be equal to the number of Maxwell elements."
      assert len(self.c) == n,\
        "Length of damping coefficients (c) should be equal to the number of Maxwell elements."
      self.sigma0 = sigma0
      self.samplingRate = samplingRate
      
    def maxwell(self, y, t):
        dydt = np.zeros(2*self.n)   
        F = y[self.n:]
        for i in range(self.n):
            dxdt = np.mean(self.velocity) - F[i] / self.c[i]
            dFdt = self.k[i] * dxdt
            dydt[i] = dxdt
            dydt[self.n + i] = dFdt
        F_total = np.sum(F)
        if np.abs(F_total) < self.sigma0:
            dydt[self.n:] = 0
        return dydt

    def computeFrictionForce(self):
        timeSpan = (len(self.velocity) - 1) / self.samplingRate
        t = np.linspace(0, timeSpan, len(self.velocity))
        initial_conditions = np.zeros(2*self.n)
        y = odeint(self.maxwell, initial_conditions, t)
        F = np.sum(y[:, self.n:], axis=1)
        return F