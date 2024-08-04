import sys
import os
import pinocchio as pin
import numpy as np
import logging
import matplotlib.pyplot as plt 
import seaborn as sns 

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dynamics import Robot
from utils import RMSE

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Regressor:
    
    def __init__(self, robot:Robot=None) -> None:
        if robot is None:
            self.robot = Robot()
        else:
            self.robot= robot  
        self.add_col = 4
        self.param_vector_max_size = (10 + self.add_col) * self.robot.model.nv
        
    def computeBasicRegressor(self,q:np.ndarray=None,v:np.ndarray=None,a:np.ndarray=None):

        id_inertias=[]
        for jj in range(len(self.robot.model.inertias.tolist())):
            if self.robot.model.inertias.tolist()[jj].mass !=0 :
                id_inertias.append(jj)
        nv= self.robot.model.nv
        W = np.zeros((nv, (10+self.add_col)*nv))
        W_mod = np.zeros((nv, (10+self.add_col)*nv))
    
        W_temp = pin.computeJointTorqueRegressor(self.robot.model, self.robot.data, q, v, a)
        for j in range(W_temp.shape[0]):
            W[j, 0 : 10 * nv] = W_temp[j, :]

            if self.robot.params['identification']['problem_params']['has_friction']:
                W[j, 10 * nv + 2 * j] = v[j]  # fv
                W[j, 10 * nv + 2 * j + 1] = np.sign(v[j])  # fs
            else:
                W[j , 10 * nv + 2 * j] = 0  # fv
                W[j, 10 * nv + 2 * j + 1] = 0  # fs
            if self.robot.params['identification']['problem_params']['has_actuator']:
                W[j, 10 * nv + 2 * nv + j] = a[j]  # ia
            else:
                W[j, 10 * nv + 2 * nv + j] = 0  # ia
            if self.robot.params['identification']['problem_params']['has_joint_offset']:
                W[j, 10 * nv + 2 * nv + nv + j] = 1  # off
            else:
                W[j, 10 * nv + 2 * nv + nv + j] = 0  # off
        for k in range(nv):
            W_mod[:, (10 + self.add_col) * k + 9] = W[:, 10 * k + 0]  # m
            W_mod[:, (10 + self.add_col) * k + 8] = W[:, 10 * k + 3]  # mz
            W_mod[:, (10 + self.add_col) * k + 7] = W[:, 10 * k + 2]  # my
            W_mod[:, (10 + self.add_col) * k + 6] = W[:, 10 * k + 1]  # mx
            W_mod[:, (10 + self.add_col) * k + 5] = W[:, 10 * k + 9]  # Izz
            W_mod[:, (10 + self.add_col) * k + 4] = W[:, 10 * k + 8]  # Iyz
            W_mod[:, (10 + self.add_col) * k + 3] = W[:, 10 * k + 6]  # Iyy
            W_mod[:, (10 + self.add_col) * k + 2] = W[:, 10 * k + 7]  # Ixz
            W_mod[:, (10 + self.add_col) * k + 1] = W[:, 10 * k + 5]  # Ixy
            W_mod[:, (10 + self.add_col) * k + 0] = W[:, 10 * k + 4]  # Ixx

            W_mod[:, (10 + self.add_col) * k + 10] = W[:, 10 * nv + 2 * nv + k]       # ia
            W_mod[:, (10 + self.add_col) * k + 11] = W[:, 10 * nv + 2 * k]            # fv
            W_mod[:, (10 + self.add_col) * k + 12] = W[:, 10 * nv + 2 * k + 1]        # fs
            W_mod[:, (10 + self.add_col) * k + 13] = W[:, 10 * nv + 2 * nv + nv + k]  # off
            
        return W_mod 
        
        
    def computeFullRegressor(self,q:np.ndarray=None,v:np.ndarray=None,a:np.ndarray=None):
        """ 
        Compute the Regressor matrix of the robot 
        This function builds the basic regressor of the 10(+4) parameters
        'Ixx','Ixy','Ixz','Iyy','Iyz','Izz','mx','my','mz','m'+ ('ia','fs','fv') 
        
        Args:
            - q: (ndarray) a configuration position vector 
            - v: (ndarray) a configuration velocity vector  
            - a: (ndarray) a configutation acceleration vector
            
        Returns:
             - W_mod: (ndarray) basic regressor for 10(+4) parameters 
                    ( NSamples * ndof, ( 10 + add_col ) * ndof) 
        """
        N = len(q) 
        id_inertias=[]
        for jj in range(len(self.robot.model.inertias.tolist())):
            if self.robot.model.inertias.tolist()[jj].mass !=0 :
                id_inertias.append(jj)
        nv= self.robot.model.nv
        W = np.zeros((N*nv, (10+self.add_col)*nv))
        W_mod = np.zeros([N*nv, (10+self.add_col)*nv])
        for i in range(N):
            W_temp = pin.computeJointTorqueRegressor(
                self.robot.model, self.robot.data, q[i, :], v[i, :], a[i, :]
            )
            for j in range(W_temp.shape[0]):
                W[j * N + i, 0 : 10 * nv] = W_temp[j, :]

                if self.robot.params['identification']['problem_params']['has_friction']:
                    W[j * N + i, 10 * nv + 2 * j] = v[i, j]  # fv
                    W[j * N + i, 10 * nv + 2 * j + 1] = np.sign(v[i, j])  # fs
                else:
                    W[j * N + i, 10 * nv + 2 * j] = 0  # fv
                    W[j * N + i, 10 * nv + 2 * j + 1] = 0  # fs
                if self.robot.params['identification']['problem_params']['has_actuator']:
                    W[j * N + i, 10 * nv + 2 * nv + j] = a[i, j]  # ia
                else:
                    W[j * N + i, 10 * nv + 2 * nv + j] = 0  # ia
                if self.robot.params['identification']['problem_params']['has_joint_offset']:
                    W[j * N + i, 10 * nv + 2 * nv + nv + j] = 1  # off
                else:
                    W[j * N + i, 10 * nv + 2 * nv + nv + j] = 0  # off
        for k in range(nv):
            W_mod[:, (10 + self.add_col) * k + 9] = W[:, 10 * k + 0]  # m
            W_mod[:, (10 + self.add_col) * k + 8] = W[:, 10 * k + 3]  # mz
            W_mod[:, (10 + self.add_col) * k + 7] = W[:, 10 * k + 2]  # my
            W_mod[:, (10 + self.add_col) * k + 6] = W[:, 10 * k + 1]  # mx
            W_mod[:, (10 + self.add_col) * k + 5] = W[:, 10 * k + 9]  # Izz
            W_mod[:, (10 + self.add_col) * k + 4] = W[:, 10 * k + 8]  # Iyz
            W_mod[:, (10 + self.add_col) * k + 3] = W[:, 10 * k + 6]  # Iyy
            W_mod[:, (10 + self.add_col) * k + 2] = W[:, 10 * k + 7]  # Ixz
            W_mod[:, (10 + self.add_col) * k + 1] = W[:, 10 * k + 5]  # Ixy
            W_mod[:, (10 + self.add_col) * k + 0] = W[:, 10 * k + 4]  # Ixx

            W_mod[:, (10 + self.add_col) * k + 10] = W[:, 10 * nv + 2 * nv + k]       # ia
            W_mod[:, (10 + self.add_col) * k + 11] = W[:, 10 * nv + 2 * k]            # fv
            W_mod[:, (10 + self.add_col) * k + 12] = W[:, 10 * nv + 2 * k + 1]        # fs
            W_mod[:, (10 + self.add_col) * k + 13] = W[:, 10 * nv + 2 * nv + nv + k]  # off
            
        return W_mod

    def computeBasicSparseRegressor(self,q,v,a):
        """ the torque of joint i do not depend on the torque of joint i-1 """
        W = self.computeBasicRegressor(q,v,a)
        for ii in range(W.shape[0]):
            for jj in range(W.shape[1]):
                if ii < jj:
                    W[ii,jj] = 0
        return W
        
    def computeReducedRegressor(self,q,v,a,tol=1e-6):
        """ 
        Eliminates columns which has L2 norm smaller than tolerance.
        Args: 
            - W: (ndarray) joint torque regressor
            - tol_e: (float) tolerance
        Returns: 
            - Wred: (ndarray) reduced regressor
        """
        W = self.computeFullRegressor(q,v,a)
        col_norm = np.diag(np.dot(np.transpose(W), W))
        idx_e = []
        for i in range(col_norm.shape[0]):
            if col_norm[i] < tol:
                idx_e.append(i)
        idx_e = tuple(idx_e)
        Wred = np.delete(W, idx_e, 1)
        return Wred 
    
    def computeRegressionCriterion(self,torque,q,v,a,x)->float:
        """ Compute the Regression error model : ε = τ - W.Θ """
        if np.ndim(x) !=1:
            logger.error('regression vector should be 1 dimeontional !')
        if x.size != self.param_vector_max_size:
            logger.error(f'x array length msismatch expected {self.param_vector_max_size}!')
        if torque.size !=  self.robot.model.nq:
            logger.error('error in torques size !')
        W  = self.computeBasicRegressor(q,v,a)
        reg_err = torque - np.dot(W,x)
        return np.linalg.norm(reg_err)
        
    
    def computeDifferentialRegressor(self, q, v, a, x,dx=1e-2):
        """ 
        This function differentiates the computeIdentificationModel of the class robot.
        Assuming the model is not linear with respect to parameter vector x:
            τ = f(q, qp, qpp, x)
        Args:
            - q: ndarray, joint positions
            - v: ndarray, joint velocities
            - a: ndarray, joint accelerations
            - dx: float, small perturbation for finite difference
        Returns: 
            - W: ndarray, (NSamples*ndof, NParams)  regressor matrix
        """
        nx = np.size(x)
        N = len(q)
        W = np.zeros((N*self.robot.model.nq, nx)) 
        self.robot.setIdentificationModelData(q, v, a)
        tau = self.robot.computeIdentificationModel(x)
        tau = tau.flatten()
    
        for i in range(nx):
            x_dx = np.copy(x)
            x_dx[i] += dx
            tau_dx = self.robot.computeIdentificationModel(x_dx)
            tau_dx = tau_dx.flatten()
            diff = (tau_dx - tau) / dx
    
            if np.any(np.isnan(diff)) or np.any(np.isinf(diff)):
                diff[np.isnan(diff)] = 0
                diff[np.isinf(diff)] = 0
            W[:, i] = diff
        return W
    
    def computeDifferentiationError(self, q, v, a, x,dx=1e-3):
        """ retun the gradient differentiation error """
        self.robot.setIdentificationModelData(q, v, a)
        f = self.robot.computeIdentificationModel(x)
        flin = self.computeDifferentialRegressor(q,v,a,x,dx) @ x
        flin = np.reshape(flin, (-1, self.robot.model.nq))
        err_per_time = RMSE(flin, f,axis=1)
        return np.sqrt(np.mean(err_per_time**2))
    
    def addJointOffset(self,q,v,a,param):
        if self.robot.params['identification']['problem_params']["has_joint_offset"]:
            logger.error('Dynamics Engine : Robot has no joint offsets. ')
            return 
        W = self.computeBasicRegressor(q,v,a)
        N = len(q)  
        nv = self.robot.model.nv
        add_col = 4
        for k in range(nv):
            W[:, (10 + add_col) * k + 13] = 1
        return W
    
    def addActuatorInertia(self, q, v, a, param):
        if self.robot.params['identification']['problem_params']["has_friction"]:
            N = len(q)  
        W = self.computeBasicRegressor(q,v,a)
        nv = self.robot.model.nv
        for k in range(nv):
            W[:, (10 + self.add_col) * k + 10] = a
        return W
    
    def addFriction(self,q,v,a,param):
        if self.robot.params['identification']['problem_params']["has_friction"]:
            logger.error('Dynamics Engine : Robot joints has no friction.')
            return 
        W = self.computeBasicRegressor(q,v,a)
        N = len(self.robot.model.q)  
        nv = self.robot.model.nv
        for k in range(nv):
            W[:, (10 + self.add_col) * k + 11] = self.robot.model.v
            W[:, (10 + self.add_col) * k + 12] = np.sign(self.robot.model.v)
            
        return W
    
    
    
    