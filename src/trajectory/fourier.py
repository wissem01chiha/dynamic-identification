import numpy as np
import logging 
import os
import matplotlib.pyplot as plt 
import seaborn as sns 

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from dynamics import   Regressor
from utils import plotArray

class FourierGenerator:
    """
    Ref: 
        Fourier-based optimal excitation trajectories for the dynamic identification of robots
        Kyung.Jo Park - Robotica - 2006.
        Modeling, Identification, and Control of Robots, 
    """
    def __init__(self,trajectory_params:dict) -> None:
        self.trajectory_params = trajectory_params
        
    def computeTrajectoryState(self,t:float=0,Q0=None, Qp0=None, Qpp0=None):
        """ Computes the trajectory states at time date t """
        pulsation = 2*np.pi*self.trajectory_params['frequancy']
        nbterms = self.trajectory_params['nbfourierterms']
        ndof = self.trajectory_params['ndof']
        Q = np.zeros(ndof) if Q0 is None else np.array(Q0)
        Qp = np.zeros(ndof) if Qp0 is None else np.array(Qp0)
        Qpp = np.zeros(ndof) if Qpp0 is None else np.array(Qpp0)

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
        return Q, Qp, Qpp
    
    def computeFullTrajectory(self, ti: float, tf: float,Q0=None, Qp0=None, Qpp0=None):
        """Computes the full trajectory data between ti and tf """
        ndof = self.trajectory_params['ndof']
        nb_traj_samples = self.trajectory_params['samples']
        time = np.linspace(ti, tf, nb_traj_samples)
        Q = np.zeros((nb_traj_samples,ndof))
        Qp = np.zeros((nb_traj_samples,ndof))
        Qpp = np.zeros((nb_traj_samples,ndof))
        for i in range(nb_traj_samples):
            Q[i,:], Qp[i,:], Qpp[i,:] = self.computeTrajectoryState(time[i],Q0,Qp0,Qpp0)
            
        return Q, Qp, Qpp
    
    def computeTrajectoryCriterion(self,ti:float,tf:float,x,Q0=None,Qp0=None,Qpp0=None)->float:
        """
        Compute the trajectory identification criteria.
        The criteria mesures how the target x parmters changes du to a variation in W or
        in mesured torques 
        for a linear or /linearized system τ ≈ W(q,qp,qpp,x0)x
        we use the Gramian matrix in calculations WTW wich is sysmetric square the cond number 
        expressed as lamdamax/lamdmin
        """
        reg = Regressor()
        Q, Qp, Qpp = self.computeFullTrajectory(ti, tf, Q0, Qp0, Qpp0) 
        W = reg.computeDifferentialRegressor(Q, Qp, Qpp,x)
        WTW = W.T @ W 
        eig_max = np.max(np.linalg.eigvals(WTW))
        C = np.abs(np.abs(eig_max))
        err = reg.computeDiffError(Q,Qp,Qpp,x)
        print(err)
        return C
    
    def computeTrajDiffError(self,ti:float,tf:float,x,Q0=None,Qp0=None,Qpp0=None):
        reg = Regressor()
        Q, Qp, Qpp = self.computeFullTrajectory(ti, tf, Q0, Qp0, Qpp0)
        err = reg.computeDiffError(Q,Qp,Qpp,x)
        return err

    def computeCriterionVsFrequency(self,fmin,fmax,ti,tf,x,Q0,Qp0,Qpp0,fsamples:int=1):
        """ compute the criteria values for a base frequancy exitation value """
        fhz = np.linspace(fmin,fmax,fsamples)
        C = np.zeros_like(fhz)
        logger.info(f'processing of {fsamples} sample points')
        for i in range(fhz.size):
            self.trajectory_params['frequancy'] = fhz[i]
            C[i] = self.computeTrajectoryCriterion(ti,tf,x,Q0,Qp0,Qpp0)  
                    
        return fhz, C
    
  
    def computeTrajectoryIdentifiability(self,ti,tf,torque,q,qp,qpp,x):
        """ 
        evaluates the rgression criteria ε(qi,qpi,qppi,x) , en fonction du trajectory certain computed 
        pervoiously C(qi,qpi,qppi) for fixed x system paramter vector :
        > the traj is identificable if the crietria is minimal in the most of time steps 
        TODO : find the rlation betwen the rgression crirtia evolution and the trajectory C over time t
        """
        reg = Regressor()
        N = len(q)
        J = np.empty(N)
        eps = np.empty(N)
        for i in range(N):
            J[i] = self.computeTrajectoryCriterion(ti,tf,q[i,:],qp[i,:],qpp[i,:])
            eps[i] = reg.computeRegressionCriterion(torque[i,:],q[i,:],qp[i,:],qpp[i,:],x)
        return  J, eps
    
    
    
    
    def computeProba(self,eps_n,J_n, bound):
        indices = np.where(J_n <= bound)[0]
        eps_n_= np.sum(np.abs(eps_n[indices]) <= 0.5)
        P = eps_n_ / len(eps_n)
        
        return P
    
    
    
    def visualizeTrajectoryIdentifiability(self, ti, tf, torque, q, qp, qpp, x):    
        """
        Visualize the trajectory identifiability criteria
        
        modeled by the probobilty function:
        f(x) = P( abs((ε - εm)/σε) < 0.05 || abs((J - Jm)/σJ) < x)  where x > 0
        """
        J, eps = self.computeTrajectoryIdentifiability(ti, tf, torque, q, qp, qpp, x)
        J_n = np.abs((J-np.mean(J))/np.std(J))
        eps_n =np.abs((eps-np.mean(eps))/np.std(eps))
        
        thresh = np.linspace(0,3,40)
        P = np.zeros_like(thresh)
        for i in range(len(thresh)):
            P[i] = self.computeProba(eps_n,J_n,thresh[i])
        
        plt.figure(figsize=(12, 6))
        sns.lineplot(x=thresh, y= P,label='Propab vs thresh',linewidth=0.7)
        plt.title('Trajectory Identifiability')
        plt.xlabel('δ')
        plt.ylabel('P')
        plt.legend()
    
    def computeTrajectoryConstraints(self,qmax,qmin,qpmax,qpmin,qppmin,qppmax,\
        ti, tf, q0=None, qp0= None, qpp0=None):
        """ 
        > Computes the trajectory constraintes 
        """
        nb_traj_samples = self.trajectory_params['samples']
        ndof = self.trajectory_params['ndof']

        joint_state = np.zeros(3 * ndof* nb_traj_samples)
        cartesian_state1 = np.zeros(nb_traj_samples)
        time = np.linspace(ti, tf, nb_traj_samples)

        for i in range(nb_traj_samples):
            state = self.computeTrajectoryState(time[i],Q0)
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
        
    def visualizeTrajectory(self, ti,tf, Q0=None, Qp0=None, Qpp0=None, savePath=None):

        q, qp, qpp = self.computeFullTrajectory(ti,tf,Q0,Qp0,Qpp0)
        plotArray(q,'Computed Trajectory Joints Positions')
        plt.savefig(os.path.join(savePath,'computed_trajectory_positions'))
        plotArray(qp,'Computed Trajectory Joints Velocity')
        plt.savefig(os.path.join(savePath,'computed_trajectory_velocity'))
        plotArray(qpp,'Computed Trajectory Joints Accelerations')
        plt.savefig(os.path.join(savePath,'computed_trajectory_accelaertions'))
            
    def save2csv(self,ti:float,tf:float,filepath,Q0=None,Qp0=None,Qpp0=None):
        # compute a given trajectory and save it to csv file : 
        Q, Qp, Qpp = self.computeFullTrajectory(ti, tf, Q0, Qp0, Qpp0)
        format = {'fmt': '%.2f', 'delimiter': ', ', 'newline': ',\n'}
        traj_array = np.concatenate([Q,Qp,Qpp])
        np.savetxt(filepath, traj_array, **format)
        