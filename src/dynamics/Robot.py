import os
import yaml
import numpy as np
import warnings
import pinocchio as pin
from utils import Math
from viscoelastic import LuGre, MaxwellSlip, Dahl, computeViscousFrictionForce

class Robot():
    """
    Robot
    Base class for Robot models.
    
    Args:
        - urdfFilePath   :
        - configFilePath :
        - model          :
    """
    def __init__(self,q=None,v=None,a=None,configFilePath=None)->None:
        
        if configFilePath ==None:
            warnings.warn("Using default configuration.", UserWarning)
 
        dir = os.path.dirname(os.path.abspath(__file__))
        configFilePath = os.path.join(os.path.dirname(os.path.dirname(dir)), 'exemple\\kinova\\config.yml')

        if not os.path.isfile(configFilePath):
            raise ValueError(f"Config file does not exist at the provided path: {configFilePath}")
        
        with open(configFilePath, 'r') as f:
            config = yaml.safe_load(f)
            
        urdfFilePath = config.get('urdfFilePath', None)
        if urdfFilePath is None:
            raise ValueError("URDF file path not set in configuration file.")        
        if not os.path.isfile(urdfFilePath):
            raise ValueError(f"URDF file does not exist at the provided path: {urdfFilePath}")
        
        # Class Attributes
        self.urdfFilePath = urdfFilePath
        self.configFilePath = configFilePath
        self.model = pin.buildModelFromUrdf(self.urdfFilePath)
        self.data =  self.model.createData()
        fext = pin.StdVec_Force()
        for i in range(self.model.njoints):
            fext.append(pin.Force.Zero())
        if q is None:
            self.q = np.zeros(self.model.nq)
        else:
            self.q = q 
        if v is  None:
            self.v =np.zeros(self.model.nv)   
        else:
            self.v = v
        if a is None:
            self.a = np.zeros(self.model.nv)
        else:
            self.a = a   
        
                
    def massMatrix(self, q=None, scaling=0.001):
        """
        Compute the mass matrix of the robot.
        Args:
            - q : Joints position vector.
            - scaling : Float used in scaling the mass matrix to prvent singulaty
        """
        if q is None:
            q = self.q
        else:
            q = np.array(q)
            if q.size < self.model.nq:
                q = np.pad(q, (0, self.model.nq - q.size), 'constant')
            elif q.size > self.model.nq:
                raise ValueError("Position input vector maximum size is", self.model.nq)
    
        M = pin.crba(self.model, self.data, q)
        if np.linalg.cond(M) < 1e-8:
            M = M + scaling * np.eye(np.shape(M)[0])
        return M
    
    def computeGeneralizedTorques(self,q=None,qp=None,qpp=None,fext=None):
        """
        Compute the genralized Torques using the recursive netwon euler alogrithm. 
        
        Args:
            - q    : Joints position vector.
            - qp   : Joints velocity vector.
            - qpp  : Joints acceleration vector.
            - fext : Joints external forces vector.
        """
        if q is None:
            q = self.q
        else:
            q = q
        if qp is None:
            qp = self.v
        else:
            qp = qp
        if qpp is None:
            qpp = self.v
        else:
            qpp = qpp
        tau = pin.rnea(self.model, self.data, self.q, self.v, self.a, self.fext)
        return tau
    
    
    def computeCorlolisMatrix(self, qp=None,q=None,timeStep=0.001,scaling=0.01):
        """
        Compute the corlolis matrix using finite deiffernce method.
        Args:
            - qp: Joints Velocity vector.
            - q : Joints Position vector.
        Returns: 
            - C : numpy.ndarray      
        """
        if q == None:
            q =self.q
        if qp == None:
            qp= self.v
        M_t = self.massMatrix(q,scaling)
        C = np.zeros(np.shape(M_t))
        q_t_1 = Math.discreteTimeIntegral(qp,timeStep)
        self.q = q_t_1
        M_t_1= self.massMatrix(self.q,scaling)
        diff_M   = ( M_t_1 - M_t )/timeStep
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    Christoffel = 1/2*( diff_M[i, j] + diff_M[i, k] - diff_M[j,k])
                    C[i,j] = C[i,j] + Christoffel * qp[k]
                             
        return C 
    
    def computeGravityTorques(self, q=None):
        """ Computes the joints gravity torques.
        Args:
            - q :  Joints poistion vector.
        Returns:
            - tau_g : numpy.ndarray.    
        """
        if q is None:
            q = self.q
        tau_g =  pin.computeGeneralizedGravity(self.model, self.data, q)
        return tau_g
    
    def computeFrictionTorques(self, qp, tspan, sampling = 1000):
        """
        Estimates the friction torque vector in robot joints given a 
        constant joint velocity vector.
 
        Args:
            - qp       : Joints velocity vector    ( 1 * ndof )
            - tspan    : Simulation time duration  (seconds)
            - sampling : Sampling frequency        (Hz)
            
        Returns:
            tau_f      : Joints friction torques   ( numSamples * ndof )
        """
        numSamples = int(1 + tspan * sampling)
        tau_f = np.zeros((numSamples, int(self.model.nv)))
        
        with open(self.configFilePath, 'r') as f:
            config = yaml.safe_load(f) 
            frictionModel = config.get('robot_params', {}).get('friction', None) 
            
        if frictionModel == 'lugre':
            for k in range(np.size(qp)):
                model = LuGre(10,5,qp[k],0.1,0.2,0.3,tspan,1/sampling)
                tau_f[:,k] = model.computeFrictionForce() 
                
        elif frictionModel == 'maxwellSlip':
            NSamples, ndof = np.shape(qp)
            assert numSamples == NSamples,"samples number mismatch"
            assert ndof == self.model.nv,'joints velocity data input msiamtch with model dof'
            
            for k in range(ndof):
                model = MaxwellSlip(3,qp[:,k],[3,1,1],[3,1,1],0.1)
                tau_f[:,k]= model.computeFrictionForce()
                
        elif frictionModel == 'dhal':
            for k in range(np.size(qp)):
                model = Dahl(0.1,1)
                tau_f[:,k]  = model.computeFrictionForce()
        elif frictionModel == 'viscous':
            for k in range(ndof):
                tau_f[:,k] = computeViscousFrictionForce(qp[k],0.1,1)
        else:
            raise ValueError("Friction Model Not Supported Yet!")
        return tau_f 
        
    def computeStiffnessTorques(self, q=None):
        """
        Compute the torques resulting from joints torsional stiffness
        eliminated from actuator torque.

        Args:
            q (numpy.ndarray): Configuration vector.
        Returns:
            numpy.ndarray: Joints stiffness torques.
        """
        if q is None:
            q = self.q[0:7]
        with open(self.configFilePath, 'r') as f:
            config = yaml.safe_load(f)
            stiffCoeffsArray = config.get('robot_params', {}).get('stiffness', None)
        if stiffCoeffsArray is None:
            raise ValueError(\
                "Stiffness coefficients are missing in the configuration file")
        nk = len(stiffCoeffsArray)
        if nk != len(q):
            raise ValueError(\
                "Number of stiffness coefficients must be equal to the number of model joints")
        tau_s = np.zeros_like(q)
        for i in range(nk):
            tau_s[i] = stiffCoeffsArray[i] * q[i]

        return tau_s
    
    def computeTrajectoryTorques(self, q, qp, qpp, fext = None):
        """
        Compute the joint torques given the trajectory data: position, velocity,
        and accelerations with fext = 0 (no external forces) and no friction.

        Args:
            q  - (numpy ndarray)  Joint positions (NSamples, ndof)
            qp - (numpy ndarray) Joint velocities (NSamples, ndof)
            qpp - (numpy ndarray) Joint accelerations (NSamples, ndof)
            fext - (numpy ndarray) Joints external angular torques applied (Nsamples * ndof)
            
        Returns: 
            tau - numpy ndarray Joint torques  (NSamples, ndof, 6)  (3D force + 3D torque)
        """
        tau = np.zeros_like(qp)
        if fext is None :
            pinfext = pin.StdVec_Force()
            for i in range(self.model.njoints):
                pinfext.append(pin.Force.Zero())
            for t in range(q.shape[0]):
                tau_i = pin.rnea(self.model, self.data, q[t,:], qp[t,:], qpp[t,:], pinfext)
                tau[t,:] = tau_i
        else:
            for t in range(q.shape[0]):
                pinfext = pin.StdVec_Force()
                for i in range(self.model.njoints):
                    linearFext = fext[t,i-1,:3]
                    angularFext = fext[t,i-1,3:]
                    pinfext.append(pin.Force(linearFext, angularFext))
                tau_i = pin.rnea(self.model, self.data, q[t,:], qp[t,:], qpp[t,:], pinfext)
                tau[t,:] = tau_i    
        return tau
    
    def computeActuatorTorques(self, kv=11, kt=0.11):
        """
        Estimates the joints motors torque vector 
        Args:
            - kv :
            - kt :
        Returns:
        
        """

    def genralizedInertiasParams(self):
        """
        Returns the genralized inertia paramters vector
        Returns:
            - initVector (numpy.ndarry) : 
        """
        vector = np.zeros((7*self.model.nq,1))
        
        return vector
    
    def setRandomInertiaParams(self)->None:
        """ 
        set the robot links inertia paramters to random values.
        Args:
        """
    
    def updateInertiaParams(self, inertiaVector)->None:
        """
        Update the inertia parmters vector 
        """
        
    def updateFrictionParams(self, paramsVector)-> None:
        """
        Update the robot friction parameters. 
        """   
        
    def updateStiffnessParams(self, vector):
        """ Update the joints stiffness paramters """ 
        
        
        
 


