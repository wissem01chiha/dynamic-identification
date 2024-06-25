import numpy as np

class Dahl:
    """
    Dahl friction Model class base definition.
    Args:
        K: Stiffness coefficient
        C: Damping coefficient
    """
    def __init__(self, K: float, C: float) -> None:
        assert K is not None and K != 0, \
            "Viscoelastic Engine: Stiffness coefficient K must be non-null float."
        self.K = K   
        self.C = C   
        self.x = 0   
        self.v = 0   

    def computeFrictionForce(self, displacement: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        """
        Compute the friction force based on the Dahl model.
        
        Args:
            displacement (np.ndarray): displacement values.
            velocity (np.ndarray):     velocity values.
            
        Returns:
            np.ndarray: computed friction forces.
        """
        friction_forces = np.zeros_like(displacement)
        
        for i in range(1, len(displacement)):
            dx = displacement[i] - displacement[i-1]
            dv = velocity[i] - velocity[i-1]
            self.x += dx
            self.v += dv
            dF = self.K * np.sign(self.v) * (1 - np.abs(friction_forces[i-1] / self.K))
            friction_forces[i] = friction_forces[i-1] + dF
            
            friction_forces[i] -= self.C * self.v

        return friction_forces