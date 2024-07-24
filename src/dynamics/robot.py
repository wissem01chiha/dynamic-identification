import os
import numpy as np
import logging
import pinocchio as pin
from utils import discreteTimeIntegral, yaml2dict, conditionNumber, MAE
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
            logger.warning("No configuration file provided using default file.")
 
        dir = os.path.dirname(os.path.abspath(__file__))
        configFilePath = os.path.join(os.path.dirname(os.path.dirname(dir)), 'exemple/kinova/config.yml')

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
        self.q = np.zeros(self.model.nq) if q is None else q
        self.v = np.zeros(self.model.nv) if v is None else v
        self.a = np.zeros(self.model.nv) if a is None else a
        self.fext = fext
  
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
                logger.error("configuation vector size should be equal to model varibles")
            elif q.size > self.model.nq:
                logger.error("Position input vector maximum size is", self.model.nq)
        if np.isnan(q).any() or np.isinf(q).any():
            logger.error("configuration array q has NAN or INF values.")
            q = np.nan_to_num(q, nan=0.0, posinf=0.0, neginf=0.0)
        M = pin.computeMinverse(self.model, self.data, q)
        if np.isnan(M).any() or np.isinf(M).any():
            logger.error("mass matrix contain NAN or INF values.")
            M = np.nan_to_num(M, nan=0.0, posinf=0.0, neginf=0.0)
        if conditionNumber(M,1e-5):
            M = M + scaling * np.eye(np.shape(M)[0])
        return  np.linalg.inv(M) 
    
    def computeGeneralizedTorques(self,q=None,qp=None,qpp=None,fext=None):
        """
        Compute the genralized Torques using the recursive netwon euler alogrithm. 
        
        Args:
            - q    : Joints position vector. ( nq * 1 )
            - qp   : Joints velocity vector. ( nq * 1 )
            - qpp  : Joints acceleration vector. ( nq * 1 )
            - fext : Joints external forces vector. ( nq * 1 )
        """
        q = q if q is not None else self.q
        qp = qp if qp is not None else self.v
        qpp = qpp if qpp is not None else self.v
        try:
            tau = pin.rnea(self.model, self.data, self.q, self.v, self.a, self.fext)
        except Exception as e:
            logger.error('Error Occured when Computing Torques.')
        return tau

    def computeCorlolisMatrix(self, qp=None,q=None,timeStep=0.001,scaling=0.01):
        """
        Compute the corlolis matrix using finite difference method.
        Args:
            - qp: Joints Velocity vector.
            - q : Joints Position vector.
        Returns: 
            - C : numpy.ndarray 
        # TODO ensure that the divion in the in derivates are not by a zero elmenent     
        """
        if q is None:
            q =self.q
        if qp is  None:
            qp= self.v
        if np.isnan(qp).any() or np.isinf(qp).any():
            logger.error("qp array contain NAN or INF values.")
            qp = np.nan_to_num(qp, nan=0.0, posinf=0.0, neginf=0.0)
        M_t = self.computeMassMatrix(q,scaling)
        C = np.zeros(np.shape(M_t))
        q_t_1 = discreteTimeIntegral(qp,timeStep)
        self.q = q_t_1
        M_t_1= self.computeMassMatrix(self.q,scaling)
        diff_M   = ( M_t_1 - M_t )
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    christoffel = 1/2*( diff_M[i,j]+ diff_M[i, k]- diff_M[j,k])
                    C[i,j] = C[i,j] + christoffel * qp[k]       
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
    
    def computeFrictionTorques(self, qp:np.ndarray, q:np.ndarray):
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
        sampling = self.params['simulation']['sampling_frequency']
        frictionModel = self.params['robot_params']['friction'] 
        
        NSamples, ndof = np.shape(qp)
        tspan = ( NSamples -1 )/sampling
        tau_f = np.zeros_like(qp)
    
        assert ndof == self.model.nq,'Joints velocity data input msiamtch with model degree of freedom'
                
        if frictionModel == 'lugre':
            Fc = self.params['friction_params']['lugre']['Fc']
            Fs = self.params['friction_params']['lugre']['Fs']
            sigma0 = self.params['friction_params']['lugre']['sigma0']
            sigma1= self.params['friction_params']['lugre']['sigma1']
            sigma2= self.params['friction_params']['lugre']['sigma2']
            qp_m = np.zeros_like(qp)
            window_size = 10
            for k in range(ndof):
                for t in range(1, len(qp), window_size):
                    model = LuGre(Fc[k], Fs[k], qp[t,k],sigma0[k], sigma1[k], sigma2[k],\
                        t,t/(2*window_size),max(t-window_size,0))
                    F = model.computeFrictionForce() 
                    tau_f[t:min(len(qp),t+window_size),k] = np.mean(F)
                    
                
        elif frictionModel == 'maxwellSlip':
  
            for j in range(int(q.shape[1])): 
                n = self.params['friction_params']['maxwellSlip']['n']
                k = self.params['friction_params']['maxwellSlip']['k']
                c = self.params['friction_params']['maxwellSlip']['c']
                sigma0 = np.array(self.params['friction_params']['maxwellSlip']['sigma0'])
                model = MaxwellSlip(n, q[:,j], k, c, sigma0[j],sampling)
                tau_f[:,j]= model.computeFrictionForce()
                
        elif frictionModel == 'dahl':
            for k in range(ndof):
                sigma0 = self.params['friction_params']['dahl']['sigma0']
                Fs = self.params['friction_params']['dahl']['Fs']
                model = Dahl(sigma0[k], Fs[k])
                tau_f[:,k]  = model.computeFrictionForce(qp[:,k])
                
        elif frictionModel == 'viscous':
            for k in range(ndof):
                Fc = self.params['friction_params']['viscous']['Fc']
                Fs = self.params['friction_params']['viscous']['Fs']
                tau_f[:,k] = computeViscousFrictionForce(qp[:,k],Fc[k],Fs[k])
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
            logger.error("Stiffness coefficients are missing in the configuration file")
        nk = len(stiffCoeffsArray)
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
                tau_i = pin.rnea(self.model, self.data, q[t,:], qp[t,:], qpp[t,:], fext)
                tau[t,:] = tau_i    
        return tau
    
    def updateExternalForces(self, F:np.ndarray)->None:
        """update the Pinnchoi external forces vector."""
        if F.size < 6:
            logger.error("Force vector size should be >= 6")
        else:
            F= F[:6]
        pinfext = pin.StdVec_Force()
        for i in range(self.model.njoints):
            linearFext = F[:3]
            angularFext = F[3:]
            pinfext.append(pin.Force(linearFext, angularFext))
        self.fext = pinfext
        
    def computeDifferentialModel(self,q=None,qp=None,qpp=None, inertia_params=None):
        if not(inertia_params is None):
            self.updateInertiaParams(inertia_params)
        M = self.computeMassMatrix(q)
        C = self.computeCorlolisMatrix(qp,q)
        G = self.computeGravityTorques(q)
        tau_sim = np.dot(M,qpp)+ np.dot(C, qp) + G
        
        return tau_sim
    
    def setIdentificationModelData(self,q,qp,qpp)->None:
        
        self.q= q
        self.v = qp 
        self.a = qpp
    
    
    def computeIdentificationModel(self,x:np.ndarray):
        """
        This function require the setup up of the joints tarjectory parmters previlouslly 
        ie self.q, v and a should be puted in the trajectoy or it will use the default.
        initlize the robot structure with the trajectory data from begin.
        Args:
            x : paramters: [inertia, friction, stiffness, actuator, fext]
                           [13n,     5n,       n,         10n,      6]
        Returns:
            tau : model output torque (Nsamples * n )
        """ 
        if (self.q is None) or  (self.v is None) or (self.a is None) or \
        np.ndim(self.q)!=2 or np.ndim(self.v)!=2 or np.ndim(self.a) !=2 :
            logger.error('Identification Data not set Run : setIdentificationModelData()') 
            
        if np.ndim(x) != 1:
            logger.error("X should be 1-dimensional array.")
            
        fext = self.params['identification']['problem_params']['has_ external_forces']    
        friction = self.params['identification']['problem_params']['has_friction']
        motor = self.params['identification']['problem_params']['has_actuator']
        stiffness = self.params['identification']['problem_params']['has_stiffness']
        n = self.model.nq
        if not(fext):
            self.updateInertiaParams(x[0:13*n])
            tau = self.computeTrajectoryTorques(self.q,self.v,self.a)
            if friction:
                self.updateFrictionParams(x[13*n:18*n])
                tau_f = self.computeFrictionTorques(self.v,self.q)
                tau = tau + tau_f
                if stiffness:
                    self.updateStiffnessParams(x[18*n:19*n])
                    tau_s = self.computeStiffnessTorques(self.q)
                    tau = tau+tau_f-tau_s
                    if motor:
                        self.updateActuatorParams(x[19*n:29*n])
                        tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                        tau = tau_m - tau_f-tau_s
                elif motor:
                    self.updateActuatorParams(x[19*n:29*n])
                    tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                    tau = tau_m - tau_f
            else:
                if stiffness:
                    self.updateStiffnessParams(x[18*n:19*n])
                    tau_s = self.computeStiffnessTorques(self.q)
                    tau = tau -tau_s
                    if motor:
                        self.updateActuatorParams(x[19*n:29*n])
                        tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                        tau = tau_m -tau_s
                elif motor:
                    self.updateActuatorParams(x[19*n:29*n])
                    tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                    tau = tau_m 
        else:
            self.updateInertiaParams(x[0:13*n])
            self.updateExternalForces(x[29*n:29*n+6]) 
            tau = self.computeTrajectoryTorques(self.q,self.v,self.a,self.fext)    
            if friction:
                self.updateFrictionParams(x[13*n:18*n])
                tau_f = self.computeFrictionTorques(self.v,self.q)
                tau = tau + tau_f
                if stiffness:
                    self.updateStiffnessParams(x[18*n:19*n])
                    tau_s = self.computeStiffnessTorques(self.q)
                    tau = tau+tau_f-tau_s
                    if motor:
                        self.updateActuatorParams(x[19*n:29*n])
                        tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                        tau = tau_m - tau_f-tau_s
                elif motor:
                    self.updateActuatorParams(x[19*n:29*n])
                    tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                    tau = tau_m - tau_f
            else:
                if stiffness:
                    self.updateStiffnessParams(x[18*n:19*n])
                    tau_s = self.computeStiffnessTorques(self.q)
                    tau = tau -tau_s
                    if motor:
                        self.updateActuatorParams(x[19*n:29*n])
                        tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                        tau = tau_m -tau_s
                elif motor:
                    self.updateActuatorParams(x[19*n:29*n])
                    tau_m = self.computeActuatorTorques(self.q,self.v,self.a)
                    tau = tau_m    
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
        #TODO to implement 
        Compute the manipulator Base inertial parameters 
        Returns
            base_params : numpy-ndarry 
        """
        base_params_vector = 1 
        return base_params_vector
        
    def updateFrictionParams(self, new_params)-> None:
        """
        update the robot friction parameters.
        Args:
            - new_params : ndarry of size min 14 (2n) max is 35(5n)
        """   
        friction_type= self.params['robot_params']['friction']
        n =  self.model.nq
        if friction_type == 'viscous': # array of 14
            if new_params.size < 2 * n:
                logger.error(f"min parms number for friction model is {2 * n}")
                return 
            else:
                new_params = new_params[0:2*n]
            self.params['friction_params']['viscous']['Fc']= new_params[0:n]
            self.params['friction_params']['viscous']['Fs']= new_params[n:2*n]
            
        elif friction_type == 'lugre':
            if new_params.size < 5 * n: # array of 35
                logger.error(f'min params number for friction model is {5 * n}')
                return
            else:
                new_params = new_params[0:5*n]
            self.params['friction_params']['lugre']['Fc']= new_params[0:n]
            self.params['friction_params']['lugre']['Fs']= new_params[n:14]
            self.params['friction_params']['lugre']['sigma0'] = new_params[14:21]
            self.params['friction_params']['lugre']['sigma1'] = new_params[21:28]
            self.params['friction_params']['lugre']['sigma2'] = new_params[28:35]
            
        elif friction_type == 'maxwellSlip': # array of 13
            if new_params.size < 6 + n:
                logger.error(f"min params number for friction model is {6 + n}")
            else:
                new_params = new_params[0:13]
            self.params['friction_params']['maxwellSlip']['k'] = new_params[0:3]
            self.params['friction_params']['maxwellSlip']['c'] = new_params[3:6]
            self.params['friction_params']['maxwellSlip']['sigma0'] = new_params[6:13]
            
        elif friction_type == 'dahl': # array of 14
            if new_params.size < 2*n:
                logger.error(f"min params number for friction model is {2*n}")
            else:
                new_params = new_params[0:14]
            self.params['friction_params']['dahl']['sigma0'] = new_params[0:7]
            self.params['friction_params']['dahl']['Fs'] = new_params[7:14]
        
    def updateStiffnessParams(self, new_params)-> None:
        """
        Update the joints stiffness paramters.
        Args:
            new_params (numpy ndarry)
        """ 
        assert new_params.size== self.model.nq,\
            "stiffness inputs should be equal to robot joints number"
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
        Estimates the joints motors torque from position, velocity and acceleration. 
        Args:
            - q: Joints position (Nsamples * ndof)
            - qp: Joints velocity (Nsamples * ndof)
        Returns:
            - tau_m : numpy.ndarry (Nsamples *ndof)
        """
        tau_m = np.zeros_like(q)
        I = self.params['actuator_params']['inertia']
        kt = self.params['actuator_params']['kt']
        damping = self.params['actuator_params']['damping']
        for i in range(self.model.nq):
            motor_i = BLDC(I[i], kt[i], damping[i])
            tau_m[:,i] = motor_i.computeOutputTorque(q[:,i], qp[:,i], qpp[:,i])
            
        return tau_m
    
    def updateActuatorParams(self, new_params:np.ndarray)->None:
        """
        Updates the joints actuator parameters.
        Bounds for torque and current (Tmax, Imax) are exclued from update.
        
        Args:
            new_params [kt, inertia, damping, Ta, Tb, Tck] (numpy-ndarry) 1 * 10.ndof
        """
        n = self.model.nq 
        assert new_params.size == 10 * n
        self.params['actuator_params']['kt'] = new_params[0:n]
        self.params['actuator_params']['inertia'] = new_params[n:2*n]
        self.params['actuator_params']['damping'] = new_params[2*n:3*n]
        self.params['actuator_params']['Ta'] = new_params[3*n:4*n]
        self.params['actuator_params']['Tb'] = new_params[4*n:5*n]
        self.params['actuator_params']['Tck'] = new_params[5*n:10*n].reshape((n, 5))
    
    def getActuatorInertiasMatrix(self)->np.ndarray:
        I = self.params['actuator_params']['inertia']
        Im = np.eye(len(I))
        for i in range(len(I)):
            Im[i] = I[i]
        return Im
        
        
 


