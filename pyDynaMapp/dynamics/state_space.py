import os
import sys
import logging
import numpy as np 
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.signal import place_poles

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dynamics import Robot

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StateSpace:
    """Base class for state space identification models 
    
    Args:
        robot  - Manipulator Base model 
    """
    def __init__(self,urdf_file_path,config_file_path) -> None:
        self.robot = Robot(urdf_file_path,config_file_path)
            
    def computeStateMatrices(self, q_or_x: np.ndarray, qp: np.ndarray = None):
        """
        Compute the state space matrices of the robot model.

        Args:
            - q_or_x: Joints position vector `q` or system state vector `x`
            - qp: Joints velocity vector (required if q_or_x is q)

        Returns:
            - A, B, C, D matrices
        """
        n = self.robot.model.nq 
        if qp is None:
            assert q_or_x.size == 2 * n, f"state vector size should be 2 * n ({2*n}), but got ({q_or_x.size}) instead"
            q = q_or_x[:n]
            qp = q_or_x[n:2*n]
        else:
            q = q_or_x
            assert q.size == qp.size, "q and qp must have the same size"

        A = np.zeros((2*n, 2*n))
        B = np.zeros((2*n, n))
        C = np.zeros((n, 2*n))
        D = np.zeros((n, n))

        c = self.robot.computeCorlolisMatrix(qp, q)
        M = self.robot.computeMassMatrix(q)
        K = self.robot.getStiffnessMatrix()

        A[n:2*n, n:2*n] = -np.dot(np.linalg.inv(M) , c)
        A[0:n, n:2*n] = np.eye(n)
        A[n:2*n, 0:n] = -np.dot(np.linalg.inv(M) , K)
        A[0:n, 0:n] = np.zeros((n, n))
        B[n:2*n,0:n] =  np.linalg.inv(M)
        C[0:n, 0:n] = np.eye(n)

        return A, B, C, D
    
    def getStateVector(self,qp:np.ndarray,q:np.ndarray):
        """Compute the State vector give the joints poistion and velocity.
        Returns:
            x - numpy-ndaryy (2.ndof * 1)
        """
        assert np.size(qp) == np.size(q),"position and velocity vector sizes must be equal"
        assert np.all(q.shape==qp.shape),"position and velocity vector shapes must be equal"
        x = np.concatenate((q, qp), axis=0)
        assert x.ndim == 1, "The state vector x should be 1-dimensional"
        return x
    
    def updateStateVector(self,q_or_x: np.ndarray,qp_or_tau:np.ndarray,tau:np.ndarray = None,\
                          state_poles=None):
        """ 
        Compute the discrete state vector at time date t + 1 given position 
        and velocity or system state vector `x`, and torques vector at time date t. 
        """
        n = self.robot.model.nq
        T = 1/self.robot.params['simulation']['sampling_frequency'] 
        if tau is None:
            x = q_or_x
            tau = qp_or_tau
            A, B, _, _ = self.computeStateMatrices(x)
            A = self.stabilize(A,B,state_poles)
            At = (np.eye(2*n) + T*A)
            Bt =  T * B
            x_next = np.dot(At, x) + np.dot(Bt,tau)
        else:
            q = q_or_x
            qp = qp_or_tau
            assert q.size == qp.size,"Input position and velocity vectors must have the same size"
            A, B, _, _ = self.computeStateMatrices(q, qp)
            x_k = self.getStateVector(qp, q)
            A = self.stabilize(A,B,state_poles)
            At = (np.eye(2*n) + T*A)
            Bt =  T * B
            x_next = np.dot(At , x_k) + np.dot(Bt,tau)
        
        return x_next
    
    def computeStateInputVector(self,states:np.ndarray,input_torque:np.ndarray=None):
        """
        Computes the state space input torques vector U.
        the input at the time date "i"  require data of i-1,i-2,...,1 states vectors  
        Returns:
            - U numpy.ndarry ( ndof * NSamples ) 
        """
        n = self.robot.model.nq
        q = states[0:n,:].T
        qp = states[n:2*n,:].T
        tau_g = self.robot.computeGravityTorques(q)
        tau_f = self.robot.computeFrictionTorques(qp,q)
        u =  tau_f +tau_g -input_torque
        u = np.clip(u,-39,39)
        return u    
        
    def getAugmentedStateVector(self,q:np.ndarray,qp:np.ndarray,tau:np.ndarray):
        """ Compute the Augemnted state vector"""
        x= self.getStateVector(qp,q)
        z = np.concatenate(x,tau,axis=0)
        return z
    
    def computeAugmentedStateMatrices(self, q_or_x: np.ndarray,qp: np.ndarray=None):
        """
        Computes and returns the augmented state-space matrices for the 
        transformed state vector z = [x, u].

        Args:
            q_or_x (np.ndarray): State vector q or x.
            qp (np.ndarray): State derivative vector (q_dot or x_dot).

        Returns:
            tuple: Augmented state-space matrices (A_aug, B_aug, C_aug, D_aug).
        """
        A, B, C, D = self.computeStateMatrices(q_or_x,qp)
        n = A.shape[0]   
        m = B.shape[1]   
        A_aug = np.block([[A, B],[np.zeros((m, n)), np.zeros((m, m))]])
        B_aug = np.block([[B],[np.eye(m)]])
        C_aug = np.block([[C, np.zeros((C.shape[0], m))]])
        D_aug = D
    
        return A_aug, B_aug, C_aug, D_aug
        
    def computeStateEigvals(self, q_or_x:np.ndarray, qp:np.ndarray=None):
        """ Returns the dynamics system eigvalues"""
        assert q_or_x.size == 2*self.robot.model.nq
        A, _, _, _ = self.computeStateMatrices(q_or_x,qp)
        eigvals = np.linalg.eigvals(A)
        eigvals = np.linalg.eigvals(A)
            
        return eigvals 
        
    def computeReducedStateMatrices(self, q:np.ndarray, qp:np.ndarray, tau:np.ndarray):
        """ computes the new transformed system by diagonalized the state matrix A 
        """ 
        A, B, C, D = self.computeStateMatrices(q,qp)
        x = self.getStateVector(qp,q)
        xdot = self.updateStateVector(q,qp,tau)
        A_hat = np.eye(1)
        xdot_ht = 1
        x_hat = 1
        B_hat = 1
        C_hat = 1
        D_hat =1   
         
    def computeStateTransitionMatrix(self,tf, ti=0):
        """ compute the state transition matrix of the system
        Args:
            - tf : final integartion time 
            - ti: initial intergartion time 
        """ 
        A,_,_,_ = self.computeStateMatrices(q_or_x,qp)
        STM = 1
        return STM
        
    def lsim(self, x0:np.ndarray, input:np.ndarray,system_poles=None,noise='gaussian',\
        verbose:bool=False):
        """ Simulate the system response with a given input torque.
        Args:
            - x0     : initial system state. ( 2.ndof * 1 )
            - input  : Input torque to the system (NSamples  * ndof )
            - noise  : additive state noise model 
            - verbose: logging display the current iteration 
        Return:
            - states : numpy-ndarry stroing iteration states vectors. ( 2.ndof * NSamples)
        """
        n = self.robot.model.nq
        NSamples, ndof = input.shape
        assert ndof == n, "msitamtech error between degree of freedom in robot data"
        states = np.empty((2*n,NSamples))
        states[:,0] = x0
        for i in range(1,NSamples):
            if verbose : 
                logger.info(f'updating state variable = {i}/{NSamples}') 
            joint_torque = self.computeStateInputVector(states[:,:i],input[:i,:])
            states[:,i] = self.updateStateVector(q_or_x=states[:,i-1],\
                qp_or_tau=joint_torque[i-1,:],state_poles=system_poles)
            if not(noise is None):
                if noise =='gaussian':
                    states[:,i]+= self.computeGaussianNoise(2*n) 
                elif noise == 'poisson':
                    states[:,i]+= self.computePoissonNoise(2*n)
                else:
                    logger.error('Noise Model not Supported.')
            
        return states 
    
    def state_place_poles(self,gain_matrix:np.ndarray,states:np.ndarray,verbose:bool=False):
        """
        this method is extention of scipy.signals.place_poles for state depend 
        varing systems, over each sample data
            x_{k+1} = A(k)x_{k} + B(k)u_{k}
        Returns:
            AA  : numpy.ndarrry [A(1), A(2), ... A(N)] ( 2*ndof , 2*ndof*N)
        """
        n_, NSamples = states.shape
        ndof = self.robot.model.nq
        assert n_ == 2* ndof, "msitamtech error between degree of freedom in robot data"
        nn, mm = gain_matrix.shape 
        assert nn == 2*ndof and mm ==NSamples, 'gain matrix shape not suitable'
        AA = np.zeros((2*ndof,2*ndof*NSamples))
        for i  in range(1,NSamples):
            if verbose :
                logger.info(f'processing sample = {i}/{NSamples}')
            A, B,_,_ = self.computeStateMatrices(states[:,i])
            AA[:,(i-1)*2*ndof:i*2*ndof]= self.stabilize(A,B,gain_matrix[:,i-1])  
            
        return AA
    
    def linearize(self,states_matrix:np.ndarray):
        """ Computes a- equivalent LTI system by avaraging the state depend 
        Linear system. 
        Args:
            states_matrix : (2*ndof, NSamples)
        """
        ndof = self.robot.model.nq
        nn, NSamples = states_matrix.shape
        assert nn == 2*ndof, 'state matrix rows should be 2*ndof the robot'
        A_avg = np.zeros((2*ndof,2*ndof))
        B_avg = np.zeros((2*ndof,ndof))
        C_avg = np.zeros((ndof, 2*ndof))
        D_avg = np.zeros((ndof, ndof))        
        for i in range(NSamples):
            A,B,C,D = self.computeStateMatrices(q_or_x=states_matrix[:,i])
            A_avg += A
            B_avg += B
            C_avg += C
            D_avg += D
        
        return A_avg/NSamples, B_avg/NSamples, C_avg/NSamples ,D_avg/NSamples
    
    
    def stabilize(self,A,B,desired_poles:np.ndarray= None):
        """ 
        Check what ever the numerical recursive control scheme given by :
                  x(k+1) = Ax(k) + B u
        is stable or not and adjust  it if necessary within ploes placement 
        strategy.
        """
        if desired_poles is None:
            desired_poles = list(self.robot.params['state_space_params']['poles'])
        else:
            desired_poles = desired_poles.tolist()
        rank_B = np.linalg.matrix_rank(B)
        if rank_B == 0:
            logger.error("The control matrix B has rank 0")
        pole_counts = {pole: desired_poles.count(pole) for pole in set(desired_poles)}
        for pole, count in pole_counts.items():
            if count > rank_B:
                logger.error(\
            f"The pole {pole} is repeated {count} times, more than the rank of B ({rank_B}).")
        eigenvalues = np.linalg.eigvals(A)
        if np.any(np.abs(eigenvalues) < 1) :
            result = place_poles(A, B, desired_poles)
            k = result.gain_matrix 
            A_new = A - np.dot(B, k)
        else:
            A_new = A
        return A_new
    
    def computeObsMatrix(self, q_or_x:np.ndarray, qp: np.ndarray=None):
        """Compute the observaliblite matrix of the robot"""
        n = self.robot.model.nq
        A, _, C, _= self.computeStateMatrices(q_or_x, qp)
        obs_matrix = C
        for i in range(1,2*n):
            obs_matrix = np.vstack((obs_matrix, C*np.linalg.matrix_power(A,i)))        
        return obs_matrix
    
    def computeCtlbMatrix(self, q_or_x: np.ndarray, qp: np.ndarray = None):
        """Compute the controllability matrix of the robot"""
        n = self.robot.model.nq
        A, B, _, _ = self.computeStateMatrices(q_or_x, qp)
        ctlb_matrix = B
        for i in range(1, n):
            ctlb_matrix = np.hstack((ctlb_matrix, np.linalg.matrix_power(A, i) @ B))

        return ctlb_matrix
    
    def computeGaussianNoise(self,length):
        mean = self.robot.params['noise_params']['gauss']['mean']
        stdv = self.robot.params['noise_params']['gauss']['stdv']
        noise_array = np.random.normal(mean,stdv,length)
        
        return noise_array
    
    def computePoissonNoise(self,length):
        lamda = self.robot.params['noise_params']['lamda']
        noise_array = np.random.poisson(lamda,length)
        
        return noise_array
     
    def visualizeRootLocus(self, q_or_x:np.ndarray,qp:np.ndarray=None)->None:
        """ Plot the system root locus for a given trajectory."""
        gain = np.linspace(0, 0.45, 35)
        K = np.ones((7, 14))
        A, B,_,_ = self.computeStateMatrices(q_or_x,qp)
        poles = np.array([np.linalg.eigvals(A-np.dot(B,K*k)) for k in gain])
        for j in range(poles.shape[1]):
            plt.plot(np.real(poles[:,j]), np.imag(poles[:,j]), '.', markersize=3)
        plt.title('Root Locus Plot')
        plt.xlabel('Real')
        plt.ylabel('Imaginary')
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.grid(True)
         
    def visualizeStatePoles(self,q,qp):
        """Plot the system poles for a given trajectory"""
        eigenvalues = self.getStateEigvals(q,qp)
        plt.figure(figsize=(6, 4))
        plt.scatter(np.real(eigenvalues), np.imag(eigenvalues), marker='*', color='blue')
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.xlabel('Real')
        plt.ylabel('Imaginary')
        plt.title('Pole Plot')
        plt.grid(True)