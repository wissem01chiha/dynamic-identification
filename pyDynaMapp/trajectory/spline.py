import numpy as np
import matplotlib.pyplot as plt
import logging
from scipy.interpolate import CubicSpline
from ..utils import plotArray

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SplineGenerator:
  
    def __init__(self, trajectory_params) -> None:
        self.trajectory_params = trajectory_params
    
    def computeTrajectoryState(self, t, Q0=None) -> np.ndarray:
        """Computes the trajectory states at time t."""
        if Q0 is None:
            Q0 = self.trajectory_params['Q0']
        cs = CubicSpline(self.trajectory_params['time_points'], Q0)
        states = cs(t)
        return states
    
    def computeTrajectoryIdentifiability(self):
        """Evaluates the regression criteria ε(q, qp, qpp, x)."""
        epsilon = 0.0
        logger.info("Computing trajectory identifiability criteria.")
        
        
        return epsilon
    
    def computeFullTrajectory(self, ti: float, tf: float, q0=None, qp0=None, qpp0=None):
        """Computes the full trajectory between ti and tf."""
        t = np.linspace(ti, tf, 100)  
        q = self.computeTrajectoryState(t, q0)
        qp = np.gradient(q,t)  
        qpp = np.gradient(qp,t)  
        logger.info("Full trajectory computed.")
        return q, qp, qpp
    
    def computeTrajectoryCriterion(self, t_i: float, t_f: float) -> float:
        """Computes the trajectory criterion between t_i and t_f."""
        q, qp, qpp = self.computeFullTrajectory(t_i, t_f)
        criterion = np.sum(q**2 + qp**2 + qpp**2)  
        logger.info(f"Trajectory criterion computed: {criterion}")
        
        return criterion
    
    def computeTrajectoryConstraints(self,qmax,qmin,qpmax,qpmin,qppmin,qppmax,ti,tf,q0=None,qp0=None,qpp0=None):
        """Ensures trajectory meets specified constraints."""
        q, qp, qpp = self.computeFullTrajectory(ti, tf, q0, qp0, qpp0)
        is_within_constraints = (
            np.all(q >= qmin) and np.all(q <= qmax) and
            np.all(qp >= qpmin) and np.all(qp <= qpmax) and
            np.all(qpp >= qppmin) and np.all(qpp <= qppmax)
        )
        logger.info(f"Trajectory constraints check: {is_within_constraints}")
        return is_within_constraints
    
    def visualizeTrajectory(self, ti, tf, Q0=None, Qp0=None, Qpp0=None):
        """Visualizes the computed trajectory."""
        q, qp, qpp = self.computeFullTrajectory(ti, tf, Q0, Qp0, Qpp0)
        plotArray(q, 'Computed Trajectory Joints Positions')
        plotArray(qp, 'Computed Trajectory Joints Velocities')
        plotArray(qpp, 'Computed Trajectory Joints Accelerations')
    
    def saveTrajectory2file(self, filename='trajectory.txt'):
        """Saves the computed trajectory to a file."""
        q, qp, qpp = self.computeFullTrajectory(0, 1)  # Assuming some default time range
        with open(filename, 'w') as f:
            for i in range(len(q)):
                f.write(f"{q[i]}\t{qp[i]}\t{qpp[i]}\n")
        logger.info(f"Trajectory saved to {filename}")
