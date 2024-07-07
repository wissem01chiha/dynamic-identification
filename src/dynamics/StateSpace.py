import logging
import numpy as np 
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.signal import place_poles
from dynamics import Robot
 
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StateSpace:
    """
    Base class for state space identification models 
    
    Args:
        robot  - Manipulator Base model 
    """
    def __init__(self,robot:Robot=None) -> None:
        if robot is None:
            self.robot = Robot()
        else:
            self.robot =robot
            
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
        assert np.all(q.shape==qp.shape), "position and velocity vector shapes must be equal"
        x = np.concatenate((q, qp), axis=0)
        assert x.ndim == 1, "The state vector x should be 1-dimensional"
        return x
    
    def updateStateVector(self, q_or_x: np.ndarray, qp_or_tau: np.ndarray, tau: np.ndarray = None):
        """ 
        Compute the discrete state vector at time date t + 1 given position 
        and velocity or system state vector `x`, and torques vector at time date t. 
        """
        n = self.robot.model.nq
        if tau is None:
            x = q_or_x
            tau = qp_or_tau
            A, B, _, _ = self.computeStateMatrices(x)
            A = self.stabilize(A,B)
            x_next = np.dot(A,x) + np.dot(B,tau)
        else:
            q = q_or_x
            qp = qp_or_tau
            assert q.size == qp.size, "Input position and velocity vectors must have the same size"
            A, B, _, _ = self.computeStateMatrices(q, qp)
            x_k = self.getStateVector(qp, q)
            A = self.stabilize(A,B)
            x_next = np.dot(A , x_k) + np.dot(B,tau)
        
        return x_next
    
    def computeStateInputVector(self, q:np.ndarray, qp:np.ndarray, qpp:np.ndarray, \
        tau:np.ndarray=None, noise:bool=False):
        """Computes the state space input torques vector U. """
        n = self.robot.model.nq
        tau_g = self.robot.computeGravityTorques(q)
        tau_f = self.robot.computeFrictionTorques(qp)
        tau_m = self.robot.computeActuatorTorques(q,qp,qpp)
        if tau is None:
            tau = tau_m
        u = tau_f + tau_g - tau
        # adress the problem where some values in u are grater than others(outliers) wich can lead to 
        # numercial issues 
        return u    
        
    def computeObsMatrix(self, q_or_x:np.ndarray, qp: np.ndarray=None):
        """Compute the observaliblite matrix of the robot"""
        n = self.robot.model.nq
        if qp is None:
            A, _, C, _ = self.computeStateMatrices(q_or_x)
        else:
            A, _, C, _ = self.computeStateMatrices(q_or_x, qp)
        obs_matrix = C
        for i in range(1, 2*n):
            obs_matrix = np.vstack((obs_matrix, C * np.linalg.matrix_power(A, i)))        
        return obs_matrix
    
    def computeCtlbMatrix(self, qp:np.ndarray, q:np.ndarray):
        """ Compute the controllabilty matrix of the robot"""
        n = self.robot.model.nq
        ctlb_matrix = B
        A, B, _, _ = self.computeStateMatrices(q,qp)
        return ctlb_matrix
    
    def getAugmentedStateVector(self, q:np.ndarray, qp :np.ndarray, tau:np.ndarray):
        """ Compute the Augemnted state vector"""
        x= self.getStateVector(qp,q)
        z = np.concatenate(x,tau,axis=0)
        return z
    
    def computeAugemntedStateMatrices(self, q_or_x:np.ndarray, qp:np.ndarray):
        """ Computes and retuns """
        A, B, C, D = self.computeStateMatrices(q_or_x)
        
    
    def getStateEigvals(self, q_or_x:np.ndarray, qp:np.ndarray=None):
        """ Returns the system eigvalues"""
        if qp is None: 
            assert q_or_x.size == 2*self.robot.model.nq
            A, _, _, _ = self.computeStateMatrices(q_or_x)
            eigvals = np.linalg.eigvals(A)
        else:
            A, _, _, _ = self.computeStateMatrices(q_or_x, qp)
            eigvals = np.linalg.eigvals(A)
            
        return eigvals 
        
    def computeReducedStateMatrices(self, q:np.ndarray, qp:np.ndarray, tau:np.ndarray):
        """ 
        computes the new transformed system by diagonalized the state matrix
        A 
        """ 
        A, B, C, D = self.computeStateMatrices(q,qp)
        x = self.getStateVector(qp,q)
        xdot =self.updateStateVector(q,qp,tau)
        A_hat = np.eye(1)
        xdot_ht = 1
        x_hat =1
        B_hat = 1
        C_hat = 1
        D_hat =1    
    
    def lsim(self, x0:np.ndarray, input:np.ndarray):
        """ simualte the ss system by computing the full analytic equation and iteragte it """
        states = 0
        return states
    
    
    def simulate(self, x0:np.ndarray, input:np.ndarray=None, noise=None, verbose:bool=False):
        """ 
        Simulate the system response with a given input torque.
        Args:
            - x0     : initial system state. ( 2.ndof * 1 )
            - input  : Input torque to the system (NSamples  * ndof )
            - noise  : 
            - verbose: logging display the current iteration 
        Returns:
            - states : numpy-ndarry stroing iteration states vectors. ( 2.ndof * NSamples)
        """
        NSamples, ndof = input.shape
        n = self.robot.model.nq
        assert ndof == n, "ndof msitamtech"
        states = np.zeros((2*n,NSamples))
        states[:,0] = x0
        for i in range(1,NSamples):
            if verbose : 
                logger.info(f'Updating state variable = {i}')
            states[:,i] = self.updateStateVector(states[:,i-1],input[i,:])
            if not(noise is None):
                if noise =='gaussian':
                    states[:,i]+= np.random.normal(0, 0.08, 2*n) 
                elif noise == 'poisson':
                    states[:,i]+= np.random.poisson(0.08, 2*n)
                else:
                    logger.error('Noise Model not Supported.')
            
        return states 
    
    def linearize(self,q:np.ndarray, qp:np.ndarray, noise:bool=False):
        """linearize by finite diffrence method the state-depend dynamics matrices.
        approximate the matrixe A and B variation to fourier or taylor series.
        to use LPV system utilities
        """
        A,_,_,_ = self.computeStateMatrices(q,qp)
        
        return 
    
    def computeluenbergerObserver(self):
        """ Computes the luenberger observer for the default sytem"""
        L = 0 
        return L 
    
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
                logger.error(f"The pole {pole} is repeated {count} times, more than the rank of B ({rank_B}).")
        eigenvalues = np.linalg.eigvals(A)
        if np.any(np.abs(eigenvalues) < 1) :
            result = place_poles(A, B, desired_poles)
            k = result.gain_matrix 
            A_new = A - np.dot(B, k)
        else:
            A_new = A
        return A_new
    
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
        
    def computeGaussianNoise(self,length):
        mean = self.robot.params['noise_params']['gauss']['mean']
        stdv = self.robot.params['noise_params']['gauss']['stdv']
        noise_array = np.random.normal(mean,stdv,length)
        
        return noise_array
    
    def computePoissonNoise(self,length):
        lamda = self.robot.params['noise_params']['lamda']
        noise_array = np.random.poisson(lamda,length)
        
        return noise_array 