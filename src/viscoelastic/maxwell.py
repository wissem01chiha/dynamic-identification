import numpy as np

class Maxwell:
    """
    Maxwell-Voight contact model class.
    
    Args:
        sigma0 (float): Initial stress value.
        eta (float): Viscosity parameter.
        E (float): Elastic modulus.
    """

    def __init__(self, sigma0: float, eta: float, E: float) -> None:
        """Initialize the Maxwell-Voight model parameters."""
        self.sigma0 = sigma0
        self.eta = eta
        self.E = E

    def stress(self, strain: np.ndarray, strain_rate: np.ndarray) -> np.ndarray:
        """
        Calculate the stress based on strain and strain rate using the Maxwell-Voight model.

        Args:
            strain (np.ndarray): Strain values.
            strain_rate (np.ndarray): Strain rate values.

        Returns:
            np.ndarray: Calculated stress values.
        """
        return self.sigma0 + self.E * strain + self.eta * strain_rate

    def simulate(self, time: np.ndarray, strain: np.ndarray) -> np.ndarray:
        """
        Simulate the stress response over time for a given strain history.

        Args:
            time (np.ndarray): Time values.
            strain (np.ndarray): Strain values over time.

        Returns:
            np.ndarray: Stress values over time.
        """
        strain_rate = np.gradient(strain, time)
        stress = self.stress(strain, strain_rate)
        return stress