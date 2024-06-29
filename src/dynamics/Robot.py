import os
import numpy as np
import logging
import pinocchio as pin
from utils import discreteTimeIntegral, yaml2dict
from viscoelastic import LuGre, MaxwellSlip, Dahl, computeViscousFrictionForce
from models import BLDC

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
 
class Robot():
    """
    Robot
    Base class for Robot models.
    
    Args:
        - urdfFilePath   : Manipulator 
        - configFilePath : Manipulator configuration file path 
        - model          : Pinocchio multibody model of the manipulator
        - params         : Model static paramters
        - data           : Pinocchio multibody data model
        - q              : Joints position vector 
        - v              : Joints velocity vector 
        - a              : Joints acceleration vector.
        
    """
    def __init__(self,q=None,v=None,a=None,configFilePath=None)->None:
        
        if configFilePath ==None:
            logger.warning("No configuration file provided using default.")
 
        dir = os.path.dirname(os.path.abspath(__file__))
        configFilePath = os.path.join(os.path.dirname(os.path.dirname(dir)), 'exemple\\kinova\\config.yml')

        if not os.path.isfile(configFilePath):
            logger.error(f"Config file does not exist at the provided path: {configFilePath}")
        
        self.params = yaml2dict(configFilePath)
        
        urdfFilePath = self.params['urdfFilePath']  
        if urdfFilePath is None:
            logger.error("URDF file path not set in configuration file.")        
        if not os.path.isfile(urdfFilePath):
            logger.error(f"URDF file does not exist at the provided path: {urdfFilePath}")
        
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
              
    def computeMassMatrix(self, q=None, scaling=0.001):
        """
        Compute the mass matrix of the robot.
        Args:
            - q : Joints position vector.
            - scaling : Float used in scaling the mass matrix to prevent singulaty
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
        Compute the corlolis matrix using finite difference method.
        Args:
            - qp: Joints Velocity vector.
            - q : Joints Position vector.
        Returns: 
            - C : numpy.ndarray      
        """
        if q is None:
            q =self.q
        if qp is  None:
            qp= self.v
        M_t = self.computeMassMatrix(q,scaling)
        C = np.zeros(np.shape(M_t))
        q_t_1 = discreteTimeIntegral(qp,timeStep)
        self.q = q_t_1
        M_t_1= self.computeMassMatrix(self.q,scaling)
        diff_M   = ( M_t_1 - M_t )/timeStep
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    Christoffel = 1/2*( diff_M[i,j] + diff_M[i, k] - diff_M[j,k])
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
    
    def computeFrictionTorques(self, qp, q, sampling = 1000):
        """
        Estimates the friction torque vector in robot joints given a 
        constant joint velocity vector.
 
        Args:
            - qp       : Joints velocity vector    ( numSamples  * ndof )
            - tspan    : Simulation time duration  (seconds)
            - sampling : Sampling frequency        (Hz)
            
        Returns:
            tau_f      : Joints friction torques   ( numSamples * ndof )
        """
        NSamples, ndof = np.shape(qp)
        tspan = ( NSamples -1 )/sampling
        tau_f = np.zeros((NSamples, int(self.model.nv)))
        frictionModel = self.params['robot_params']['friction'] 
        
        assert ndof == self.model.nv,\
                'joints velocity data input msiamtch with model degree of freedom'
                
        if frictionModel == 'lugre':
            for k in range(ndof):
                for t in range(NSamples):
                    model = LuGre(10,5,qp[t,k],0.1,0.2,0.3,tspan,1/sampling)
                    F = model.computeFrictionForce() 
                    tau_f[t,k] = F[t]
                
        elif frictionModel == 'maxwellSlip':
            for k in range(ndof):
                model = MaxwellSlip(3,qp[:,k],[3,1,1],[3,1,1],0.1)
                tau_f[:,k]= model.computeFrictionForce()
                
        elif frictionModel == 'dahl':
            for k in range(ndof):
                
                model = Dahl(20,1)
                tau_f[:,k]  = model.computeFrictionForce(q[:,k],qp[:,k])
                
        elif frictionModel == 'viscous':
            for k in range(ndof):
                
                tau_f[:,k] = computeViscousFrictionForce(qp[:,k],0.1,1)
        else:
            logger.error("Friction Model Not Supported Yet!")
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
            q = self.q
        stiffCoeffsArray = self.params['stiffness_params']
        if stiffCoeffsArray is None:
            logger.error(\
                "Stiffness coefficients are missing in the configuration file")
        nk = len(stiffCoeffsArray)
        if nk != len(q):
            logger.error(\
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
        
    def genralizedInertiasParams(self):
        """
        Returns the genralized inertia paramters vector
        Returns:
            - inertia_vectors (numpy.ndarry) : concatenated inertia paramters of all 
                links
        """
        inertia_vectors = np.zeros((self.model.nq,13))
        for i in range(self.model.nq):
            m_i = self.model.inertias[i].mass
            c_i = self.model.inertias[i].lever.flatten()
            I_i =  self.model.inertias[i].inertia.flatten()
            inertia_vector_i = np.concatenate(([m_i], I_i, c_i))
            inertia_vectors[i,:] = inertia_vector_i
            
        inertia_vectors.flatten().reshape(-1, 1)
        return inertia_vectors
    
    def updateInertiaParams(self, inertia_vector)->None:
        """Update the inertia paramters vector"""
        assert inertia_vector.size == 13 * self.model.nq, \
            "The size of the inertia vector does not match the expected size."
        idx = 0
        for i in range(self.model.nq):
            m_i = inertia_vector[idx]
            c_i = inertia_vector[idx + 1:idx + 4]
            I_i_flat = inertia_vector[idx + 4:idx + 13]
            I_i = I_i_flat.reshape(3,3)
            self.model.inertias[i].mass = m_i
            self.model.inertias[i].lever = c_i
            self.model.inertias[i].inertia = I_i
            idx += 13
        
    def setRandomInertiaParams(self)->None:
        """set the robot links inertia paramters to random values."""
        Xphi = self.genralizedInertiasParams()
        randXphi = np.random.rand(np.shape(Xphi))
        self.updateInertiaParams(randXphi)
        
    def computeBaseInertiasParams(self):
        """  
        Compute the manipulator Base inertial parameters 
        Returns
        
        """
        base_params_vector = 1 
        return base_params_vector
        
    def updateFrictionParams(self, new_params)-> None:
        """
        Update the robot friction parameters. 
        """   
        friction_type= self.params['robot_params']['friction']
        if friction_type == 'viscous':
            assert new_params.size == 2* self.model.nq # vector of 14
            new_params = np.reshape(new_params,(2,self.model.nq))
            self.params['friction_params']['viscous']['Fc']= new_params[0,:]
            self.params['friction_params']['viscous']['Fs']= new_params[1,:]
        elif friction_type == 'lugre':
            assert new_params.size == 5* self.model.nq # vector of 35
            new_params = np.reshape(new_params,(5,self.model.nq))
            self.params['friction_params']['lugre']['Fc']= new_params[0,:]
            self.params['friction_params']['lugre']['Fs']= new_params[1,:]
            self.params['friction_params']['lugre']['sigma0'] = new_params[2,:]
            self.params['friction_params']['lugre']['sigma1'] = new_params[3,:]
            self.params['friction_params']['lugre']['sigma2'] = new_params[4,:]
            
        elif friction_type == 'maxwellSlip':
            assert new_params.size == 6 
            new_params = np.reshape(new_params,(5,self.model.nq))
            self.params['friction_parms']['maxwellSlip']['k'] = new_params[0,:]
            self.params['friction_parms']['maxwellSlip']['c'] = new_params[1,:]
        elif friction_type == 'dahl':
            assert new_params.size == 2
            self.params['friction_parms']['dahl']['k'] = new_params[0]
        
    def updateStiffnessParams(self, new_params)-> None:
        """
        Update the joints stiffness paramters.
        Args:
            new_params (numpy ndarry)
        """ 
        assert new_params.size== self.model.nq,\
            "Stiffness inputs paramters should be equal to robot joints number"
        self.params['stiffness_params'] = new_params
        
    def getStiffnessMatrix(self)->np.ndarray:
        """
        Return the diagonal stiffness matrix of the robot formed by each joint 
        stiffness factor.
        """
        matrix = np.eye(self.model.nq)
        for i in range(self.model.nq):
            matrix[i,i] = self.params['stiffness_params'][i]
        return matrix 
        
    def computeActuatorTorques(self, q, qp, qpp):
        """
        Estimates the joints motors torque vector.
        Args:
            - q:
            - qp: 
        Returns:
            - tau_m : numpy.ndarry
        """
        tau_m = np.zeros_like(q)
        I = self.params['actuator_params']['inertia']
        kt = self.params['actuator_params']['kt']
        damping = self.params['actuator_params']['damping']
        for i in range(self.model.nq):
            motor_i = BLDC(I[i], kt[i], damping[i])
            tau_m[:,i] = motor_i.computeOutputTorque(q[:,i], qp[:,i], qpp[:,i])
            
        return tau_m
    
    def updateActuatorParams(self, new_params)->None:
        """
        Updates the joints actuator parameters.
        
        Args:
        
        """
            
    
 


