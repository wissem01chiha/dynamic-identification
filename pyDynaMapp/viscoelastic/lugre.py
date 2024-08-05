import numpy as np

class LuGre:
    """
    Class to compute LuGre Friction Model 
    
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

    def __init__(self, Fc, Fs, v, sigma0, sigma1, sigma2, tspan, ts=0.001, tinit=0, z0=0.01, vs=0.1235):
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
        N = len(t)
        
        z = np.zeros(N, dtype=np.float64)
        F = np.zeros(N, dtype=np.float64)
        z[0] = self.z0

        for idx in range(1, N):
            v_safe = max(abs(self.vs), 1e-3)
            sigma0_safe = max(abs(self.sigma0), 1e-6)
            exp_input = -(self.v / v_safe) ** 2
            exp_input_clipped = np.clip(exp_input, -1e6, 1e6)
            gv = (self.Fc + (self.Fs - self.Fc) * np.exp(exp_input_clipped)) / sigma0_safe
            gv = max(gv, 1e-4)  # Ensure gv does not become too small
            
            z_dot = self.v - abs(self.v) * z[idx-1] / gv
            z[idx] = z[idx-1] + z_dot * self.ts
            if np.isnan(z[idx]) or np.isinf(z[idx]):
                z[idx] = 0
            
            F[idx] = self.sigma0 * z[idx] + self.sigma1 * z_dot + self.sigma2 * self.v
            if np.isnan(F[idx]) or np.isinf(F[idx]):
                F[idx] = 0

        return F

    def computeSteadyForce(self):
        """
        Compute the LuGre steady state friction force

        Returns:
            - Fss (float): Steady state friction force.
        """
        v_safe = max(np.abs(self.vs), 1e-6)
        exp_input = -(self.v / v_safe) ** 2
        exp_input_clipped = np.clip(exp_input, -1e6, 1e6)
        Fss = self.Fc * np.sign(self.v) + (self.Fs - self.Fc) * \
              np.exp(exp_input_clipped) * np.sign(self.v) + self.sigma2 * self.v
        return Fss
