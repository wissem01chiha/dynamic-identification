import numpy as np

from numba import cuda
import logging
import math

numba_logger = logging.getLogger('numba')
numba_logger.setLevel(logging.ERROR)

class cuLuGre:
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

    def __init__(self, Fc, Fs, v, sigma0, sigma1, sigma2, tspan, ts=0.001, tinit=0, z0=0.0001, vs=0.1235):
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

        z = np.zeros(N, dtype=np.float32)
        F = np.zeros(N, dtype=np.float32)
        z[0] = self.z0

        d_z = cuda.to_device(z)
        d_F = cuda.to_device(F)
        threads_per_block = 1
        blocks_per_grid = (N + (threads_per_block - 1)) // threads_per_block
        self._luGre_kernel[blocks_per_grid, threads_per_block](self.v, self.vs, self.Fc, self.Fs, \
            self.sigma0, self.sigma1, self.sigma2, self.ts, d_z, d_F, N)
        d_F.copy_to_host(F)
        return F
    
    @staticmethod
    @cuda.jit
    def _luGre_kernel(v, vs, Fc, Fs, sigma0, sigma1, sigma2, ts, z, F, N):
        idx = cuda.grid(1)
        if idx < N:
            v_safe = max(math.fabs(vs), 1e-3)
            sigma0_safe = max(math.fabs(sigma0), 1e-6)
            exp_input = -(v / v_safe) ** 2
            exp_input_clipped = min(max(exp_input, -1e6), 1e6)  
            gv = (Fc + (Fs - Fc) * math.exp(exp_input_clipped)) / sigma0_safe
            if gv < 1e-6:
                gv = 1e-6
            if idx > 0:
                z_dot = v - math.fabs(v) * z[idx-1] / gv
                z[idx] = z[idx-1] + z_dot * ts
                if math.isnan(z[idx]) or math.isinf(z[idx]):
                    z[idx] = 0
                F[idx] = sigma0 * z[idx] + sigma1 * z_dot + sigma2 * v
                if math.isnan(F[idx]) or math.isinf(F[idx]):
                    F[idx] = 0

    def computeSteadyForce(self):
        """
        Compute the Lugre steady state friction force

        Returns:
            - Fss (float): Steady state friction force.
        """
        v_safe = max(np.abs(self.vs), 1e-6)  # Ensure vs is not too small
        exp_input = -(self.v / v_safe) ** 2
        exp_input_clipped = np.clip(exp_input, -1e6, 1e6)  # Clipping the input to avoid overflow
        Fss = self.Fc * np.sign(self.v) + (self.Fs - self.Fc) * \
              np.exp(exp_input_clipped) * np.sign(self.v) + self.sigma2 * self.v
        return Fss
