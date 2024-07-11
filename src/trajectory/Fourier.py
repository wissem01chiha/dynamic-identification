import numpy as np
import logging 
import matplotlib.pyplot as plt 
import seaborn as sns 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from dynamics import  Robot, Regressor

class FourierGenerator:

    def __init__(self,trajectory_params:dict) -> None:
        self.trajectory_params = trajectory_params
        
    def computeTrajectoryStates(self,t,Q0=None)->np.ndarray:
        
        pulsation = 2*np.pi*self.trajectory_params['frequancy']
        nbterms = self.trajectory_params['nbfourierterms']
        ndof = self.trajectory_params['ndof']
        if Q0 is None :
            Q0 = np.zeros(ndof) 
        Q = np.array(Q0)
        Qp = np.zeros(ndof)
        Qpp = np.zeros(ndof)

        if ndof != len(Q):
            logger.error('Trajectory Generation Engine :inconsistency in ndof!')

        for i in range(ndof):
            for j in range(1, nbterms + 1):
                Aij = self.trajectory_params['Aij'][i][j - 1]
                Bij = self.trajectory_params['Bij'][i][j - 1]
                
                Cojt = np.cos(pulsation * j * t)
                Sojt = np.sin(pulsation * j * t)

                Q[i] += Aij / (pulsation * j) * Sojt - Bij / (pulsation * j) * Cojt
                Qp[i] += Aij * Cojt + Bij * Sojt
                Qpp[i] += Bij * j * Cojt - Aij * j * Sojt

            Qpp[i] = pulsation * Qpp[i]

        return np.concatenate([Q, Qp, Qpp])
    
    def computeTrajectoryCriterion(self, t_i, t_f, Q0:np.ndarray=None)->float:
        
        nb_traj_samples = self.trajectory_params['samples']
        ndof =  self.trajectory_params['ndof']
        W = np.zeros((ndof * nb_traj_samples, self.robot.param_vector_size))
        time_samples = np.linspace(t_i, t_f, nb_traj_samples)
        
        for i in range(nb_traj_samples):
            augmented_state = self.computeTrajectoryStates(time_samples[i],Q0)
            Qpp = augmented_state[:ndof]
            Qp = augmented_state[ndof:2 * ndof]
            Q = augmented_state[2 * ndof:]
            
            W[ndof * i:ndof * (i + 1), :] = getattr(self, \
                f'Regressor_Y_{self.robot.name}')(Q, Qp, Qpp, \
                    self.robot.numerical_parameters['Geometry'], \
                    self.robot.numerical_parameters['Gravity'])

        k1 = self.trajectory_params['k1']
        k2 = self.trajectory_params['k2']
        S = np.linalg.svd(W, compute_uv=False)
        sig_min = np.min(S)
        C = np.linalg.cond(np.dot(np.transpose(W),W))
        if sig_min <= np.Inf or sig_min== np.nan : 
            J = k1 * C
        J = k1 * C + k2 * 1 / sig_min
        
        return J
        
    def computeTrajectoryConstraints(self,state_max,state_min,ti,tf,Q0:np.ndarray=None):
        
        nb_traj_samples = self.trajectory_params['samples']
        ndof = self.trajectory_params['ndof']

        joint_state = np.zeros(3 * ndof* nb_traj_samples)
        cartesian_state1 = np.zeros(nb_traj_samples)
        time_samples = np.linspace(ti, tf, nb_traj_samples)

        for i in range(nb_traj_samples):
            state = self.computeTrajectoryStates(time_samples[i],Q0)
            joint_state[3 * ndof * i:3 * ndof * (i + 1)] = state
            Q = state[2 * ndof:]
            
            HT1 = getattr(self, f'HT_dh{self.robot.nbDOF}_world_{self.robot.name}')(Q, \
                self.robot.numerical_parameters['Geometry'])
            cartesian_state1[i] = -HT1[2, 3] + 0.3

        C = np.concatenate([
            joint_state - np.tile(state_max - 0.1, nb_traj_samples),
            np.tile(state_min + 0.1, nb_traj_samples) - joint_state,
            cartesian_state1
        ])
        Ceq = []
        
        return C, Ceq
        
    def visualizeTrajectory(self, title=None):
        """Plot the generated trajectory"""
        # option : display in title the condition number of the trajectory 
        plt.figure(figsize=(12, 6))
