import os 
import sys
import numpy as np
import logging 
import os
import matplotlib.pyplot as plt 
import seaborn as sns 


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from ..dynamics import   Regressor
from ..utils import plotArray

class FourierGenerator:
    """
    Base class for peroidic trajectories generation.
    
    Ref: 
        Fourier-based optimal excitation trajectories for the dynamic identification of robots
        Kyung.Jo Park - Robotica - 2006. 
    """
    def __init__(self,trajectory_params:dict) -> None:
        self.trajectory_params = trajectory_params
        
    def computeTrajectoryState(self,t:float=0,q0=None, qp0=None, qpp0=None):
        """ Computes the trajectory states at time date t """
        pulsation = 2*np.pi*self.trajectory_params['frequancy']
        nbterms = self.trajectory_params['nbfourierterms']
        ndof = self.trajectory_params['ndof']
        q = np.zeros(ndof) if q0 is None else np.array(q0)
        qp = np.zeros(ndof) if qp0 is None else np.array(qp0)
        qpp = np.zeros(ndof) if qpp0 is None else np.array(qpp0)

        if ndof != len(q):
            logger.error('Trajectory Generation Engine :inconsistency in ndof!')
        for i in range(ndof):
            for j in range(1, nbterms + 1):
                Aij = self.trajectory_params['Aij'][i][j - 1]
                Bij = self.trajectory_params['Bij'][i][j - 1]
                Cojt = np.cos(pulsation * j * t)
                Sojt = np.sin(pulsation * j * t)
                q[i] += Aij / (pulsation * j) * Sojt - Bij / (pulsation * j) * Cojt
                qp[i] += Aij * Cojt + Bij * Sojt
                qpp[i] += Bij * j * Cojt - Aij * j * Sojt
            qpp[i] = pulsation * qpp[i]
        return q, qp, qpp
    
    def computeFullTrajectory(self, ti: float, tf: float,q0=None, qp0=None, qpp0=None):
        """Computes the full trajectory data between ti and tf """

        ndof = self.trajectory_params['ndof']
        nb_traj_samples = self.trajectory_params['samples']
        time = np.linspace(ti, tf, nb_traj_samples)
        q = np.zeros((nb_traj_samples,ndof))
        qp = np.zeros((nb_traj_samples,ndof))
        qpp = np.zeros((nb_traj_samples,ndof))
        for i in range(nb_traj_samples):
            q[i,:], qp[i,:], qpp[i,:] = self.computeTrajectoryState(time[i],q0,qp0,qpp0)
            
        return q, qp, qpp
    
    def computeDifferentiationError(self,ti:float,tf:float,x,q0=None,qp0=None,qpp0=None):
        reg = Regressor()
        q, qp, qpp = self.computeFullTrajectory(ti, tf, q0, qp0, qpp0)
        err = reg.computeDifferentiationError(q,qp,qpp,x)
        return err
    
    def computeConstraintedDifferentiationError(self,ti:float,tf:float,x,q0=None,qp0=None,qpp0=None):
        reg = Regressor()
        q, qp, qpp = self._computeTrajectoryConstraints(ti,tf,1.57,-1.57,1.2,-1.2,5.2,-5.2, q0, qp0, qpp0)
        err = reg.computeDifferentiationError(q,qp,qpp,x)
        return err
    
    def computeTrajectoryError(self,x,tspan,new_traj_params=None,q0=None,qp0=None,\
                               qpp0=None,verbose=False):
      
        if not(new_traj_params is None):
            k = self.trajectory_params['ndof'] * self.trajectory_params['nbfourierterms']
            self.trajectory_params['Aij'] = np.reshape(new_traj_params[0:k],(-1,5))
            self.trajectory_params['Bij'] = np.reshape(new_traj_params[k:2*k],(-1,5))
        err = self.computeDifferentiationError(0,tspan,x,q0,qp0,qpp0)
        if verbose:
            print( f"RMSE = {err:.5f}")
         
        return err
    
    def _computeTrajectoryConstraints(self,ti,tf,qmax,qmin,qpmax,qpmin,qppmin,qppmax,\
        q0=None, qp0= None, qpp0=None):
        """ Computes the trajectory with taking constraintes into account """

        ndof = self.trajectory_params['ndof']
        nb_traj_samples = self.trajectory_params['samples']
        time = np.linspace(ti, tf, nb_traj_samples)
        q = np.zeros((nb_traj_samples,ndof))
        qp = np.zeros((nb_traj_samples,ndof))
        qpp = np.zeros((nb_traj_samples,ndof))
        for i in range(nb_traj_samples):
            q[i,:], qp[i,:], qpp[i,:] = self.computeTrajectoryState(time[i],q0,qp0,qpp0)
            q[i,:] = np.clip(q[i,:],qmin,qmax)
            qp[i,:] = np.clip(qp[i,:],qpmin,qpmax)
            qpp[i,:] = np.clip(qpp[i,:],qppmin,qppmax)

        return q, qp ,qpp
    

    def visualizeTrajectory(self, ti,tf, q0=None, qp0=None, qpp0=None, savePath=None):
        """Compute and plot a given trajectory, each variable in a single plot"""

        q, qp, qpp = self.computeFullTrajectory(ti,tf,q0,qp0,qpp0)
        plotArray(q,'Computed Trajectory Joints Positions')
        plt.savefig(os.path.join(savePath,'computed_trajectory_positions'))
        plotArray(qp,'Computed Trajectory Joints Velocity')
        plt.savefig(os.path.join(savePath,'computed_trajectory_velocity'))
        plotArray(qpp,'Computed Trajectory Joints Accelerations')
        if not(savePath is None): 
            plt.savefig(os.path.join(savePath,'computed_trajectory_accelaertions'))
            
    def save2csv(self,ti:float,tf:float,file_path,q0=None,qp0=None,qpp0=None):
        """ compute a given trajectory and save it to csv file.""" 

        q, qp, qpp = self.computeFullTrajectory(ti,tf,q0,qp0,qpp0)
        format = {'fmt': '%.2f', 'delimiter': ', ', 'newline': ',\n'}
        traj_array = np.concatenate([q,qp,qpp])
        np.savetxt(file_path, traj_array, **format)

    

































    def computeTrajectoryCriterion(self,ti:float,tf:float,x,q0=None,qp0=None,qpp0=None)->float:
        """
        Compute the trajectory identification criteria.
        The criteria mesures how the target x parmters changes du to a variation in W or
        in mesured torques 
        for a linear or /linearized system τ ≈ W(q,qp,qpp,x0)x
        we use the Gramian matrix in calculations WTW wich is sysmetric square the cond number 
        expressed as lamdamax/lamdmin.
        """
        reg = Regressor()
        q, qp, qpp = self.computeFullTrajectory(ti, tf, q0, qp0, qpp0) 
        W = reg.computeDifferentialRegressor(q, qp, qpp,x)
        WTW = W.T @ W 
        eig_max = np.max(np.linalg.eigvals(WTW))
        C = np.abs(np.abs(eig_max))
        return C
    


 

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
    

        

        