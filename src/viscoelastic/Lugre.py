import numpy as np

class LuGre:
    """
    Class to compute LuGre Friction Model.

    Params:
        - Fc (float): Coulomb friction coefficient.
        - Fs (float): Stribeck friction coefficient.
        - v (float): Joint velocity.
        - vs (float): Kinetic velocity transition.
        - sigma0 (float): Model parameter sigma0.
        - sigma1 (float): Model parameter sigma1.
        - sigma2 (float): Model parameter sigma2.
        - tinit (float): Initial simulation time.
        - ts (float): Step time simulation.
        - tspan (float): Final simulation time.
        - z0 (float): Initial value of internal state z.
    """

    def __init__(self, Fc, Fs, v,  sigma0, sigma1, sigma2,tspan,ts=0.001, tinit= 0, z0=0.0001,vs=0.1235,):
        self.Fc = Fc
        self.Fs = Fs
        self.v = v
        self.vs = vs
        self.sigma0 = sigma0
        self.sigma1 = sigma1
        self.sigma2 = sigma2
        self.tinit = tinit
        self.ts = ts
        self.tspan = tspan
        self.z0 = z0

    def computeFrictionForce(self):
        """
        Compute friction force over the simulation time span.

        Returns:
            - F (numpy.ndarray): Friction force for the given velocity.
        """
        t = np.arange(self.tinit, self.tspan + self.ts, self.ts)
        z = self.z0
        F = np.zeros_like(t)
        for j in range(len(t)):
            F[j], z = self._luGre(z, self.v)
        return F
    
    def computeSteadyForce(self):
        """
        Compute the Lugre steady state friction force
        Returns:
            - Fss (float): Steady state friction force.
        """
        Fss = self.Fc * np.sign(self.v) + (self.Fs - self.Fc) * \
        np.exp(-(self.v / self.vs) ** 2) * np.sign(self.v) + self.sigma2 * self.v
        return Fss

    def _luGre(self, z, v):
        """
        Internal method to compute LuGre model at each time step.

        Parameters:
            - z (float): Internal state z.
            - v (float): Joint velocity.

        Returns:
            - F (float): Friction force at the current time step.
            - z (float): Updated internal state z.
        """
        gv = (self.Fc + (self.Fs - self.Fc) * np.exp(-(v / self.vs) ** 2)) / self.sigma0
        z_dot = v - abs(v) * z / gv
        z = z + z_dot * self.ts
        F = self.sigma0 * z + self.sigma1 * z_dot + self.sigma2 * v
        return F, z