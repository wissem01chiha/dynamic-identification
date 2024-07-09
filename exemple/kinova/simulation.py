##########################################################################
# tghis script for robot data visulization adn the static simulation of the robt models 
# with pataerts stored in config.yml file
# all figures are saved in figure/pyDynamapp 
#
#
#
#
# TODO: replace all models computing function with 
# 'computeIdentificationModel' function from class Robot.
# Author: Wissem CHIHA ©
# 2024
##########################################################################
import sys
import os
import seaborn as sns
import matplotlib.pyplot as plt 
import numpy as np 
import logging
import time

figureFolderPath="C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/figure/kinova"
config_file_path="C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml"
src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)),'../src'))
sys.path.append(src_folder)
if not os.path.exists(figureFolderPath):
    os.makedirs(figureFolderPath)

from dynamics import Robot, StateSpace
from utils import RobotData,  plot2Arrays, plotElementWiseArray, yaml2dict, RMSE

mlogger  = logging.getLogger('matplotlib')
logging.basicConfig(level='INFO')
mlogger.setLevel(logging.WARNING)

cutoff_frequency  = 3
config_params  = yaml2dict(config_file_path)
data           = RobotData(config_params['identification']['dataFilePath'])
fildata        = data.lowPassfilter(cutoff_frequency)
kinova         = Robot()
q_f            = fildata ['position']
qp_f           = fildata['velocity']
qpp_f          = fildata['desiredAcceleration']
current_f      = fildata['current']
torque_f       = fildata['torque']

q              = data.position
qp             = data.velocity
qpp            = data.desiredAcceleration
current        = data.current
torque         = data.torque


# Visualize the recorded trajectory data of the system.
plot2Arrays(q, q_f, "true", "filtred", f"Joints Positions, cutoff frequency = {cutoff_frequency} Hz")
plt.savefig(os.path.join(figureFolderPath,'joints positions'))
plot2Arrays(qp, qp_f, "true", "filtred", f"Joints Velocity, cutoff frequency = {cutoff_frequency} Hz")
plt.savefig(os.path.join(figureFolderPath,'joints velocity'))
plot2Arrays(qpp, qpp_f, "true", "filtred", f"Joints Acceleration, cutoff frequency = {cutoff_frequency} Hz")
plt.savefig(os.path.join(figureFolderPath,'joints acceleration'))
plot2Arrays(torque, torque_f , "true", "filtred", f"Joints Torques, cutoff frequency = {cutoff_frequency} Hz")
plt.savefig(os.path.join(figureFolderPath,'joints torques'))
plot2Arrays(current, current_f , "true", "filtred", f"Joints Current, cutoff frequency = {cutoff_frequency} Hz")
plt.savefig(os.path.join(figureFolderPath,'joints current'))


# Compute and plot the RMSE between the actual RNEA model (Blast) and the 
# torque sensor output. 
rmse_joint = RMSE(torque, data.torque_rne).flatten()
rmse_time  = RMSE(torque, data.torque_rne,axis=1) 
plotElementWiseArray(rmse_joint,'Error between Blast RNEA and Sensor Torques per Joint'\
    ,'Joint Index','RMSE')
plt.savefig(os.path.join(figureFolderPath,'blast RNEA vs sensor torques'))


# Compute and plot the standard manipulator model : 
# τ = M(Θ)Θddot + C(Θ,Θp)Θp+ G(Θ) 
tau_sim = np.zeros_like(torque)
for i  in range(data.numRows):
    tau_sim[i,:] = 3*(kinova.computeDifferentialModel(q[i,:],qp[i,:],qpp[i,:]))
rmse_per_joint = RMSE(tau_sim,torque).flatten()
plotElementWiseArray(rmse_per_joint,"Standard Manipulator Model Error per Joint"\
    ,'Joint Index','RMSE')
plt.savefig(os.path.join(figureFolderPath,'standard_model_error_per_joint'))
plot2Arrays(torque_f,tau_sim,"true","simulation","Standard Manipulator Model")
plt.savefig(os.path.join(figureFolderPath,'standard model'))


# Compute and plot the standard manipulator model with friction effect:
# τ = M(Θ)Θddot + C(Θ,Θp)Θp + G(Θ) + τf
tau_f = kinova.computeFrictionTorques(qp,q)
tau_sim = np.zeros_like(torque)
for i  in range(data.numRows):
    tau_sim[i,:] = 3*(kinova.computeDifferentialModel(q[i,:],qp[i,:],qpp[i,:]) + tau_f[i,:])
model_error = RMSE(tau_sim,torque)
plot2Arrays(torque_f,tau_sim,"true","simulation","Standard manipulator model with friction")
plt.savefig(os.path.join(figureFolderPath,'standard model with friction'))


# Compute and plot the standard manipulator model with stiffness:
# τ = M(Θ)Θddot + C(Θ,Θp)Θp + [k]Θ + G(Θ) 
tau_sim = np.zeros_like(torque)
for i  in range(data.numRows):
    tau_s = kinova.computeStiffnessTorques(q[i,:])
    tau_sim[i,:] = 3*(kinova.computeDifferentialModel(q[i,:],qp[i,:],qpp[i,:]) + tau_s)
plot2Arrays(torque_f,tau_sim,"true","simulation","Standard manipulator model with stiffness")
plt.savefig(os.path.join(figureFolderPath,'standard model with stiffness'))


# Compute and plot the standard manipulator model with stiffness and friction:
# τ = M(Θ)Θddot + C(Θ,Θp)Θp + [k]Θ + G(Θ) + τf
tau_sim = np.zeros_like(torque)
tau_f = kinova.computeFrictionTorques(qp,q)
for i  in range(data.numRows):
    tau_s = kinova.computeStiffnessTorques(q[i,:])
    tau_sim[i,:] = 3*(kinova.computeDifferentialModel(q[i,:],qp[i,:],qpp[i,:]) + tau_s + tau_f[i,:])
plot2Arrays(torque_f,tau_sim,"true","simulation","Standard model with stiffness and friction")
plt.savefig(os.path.join(figureFolderPath,'standard model with stiffness & friction'))



# Compute an plot the system state space model simulation
# x(k+1) = A(x) x(k) + B(x) u(k)
# y(k)   = C x(k)
"""  
kinova_ss = StateSpace(kinova)
tau_ss = torque
x0 = kinova_ss.getStateVector(qp[0,:],q[0,:])
states = kinova_ss.simulate(x0,tau_ss[:30000:50,:])
plot2Arrays(MAE(0.001*np.transpose(states[7:14,:]),30),qp[:30000:50,:],'state','true',\
    'Joints Velocity State Model Simulation')
plt.savefig(os.path.join(figureFolderPath,'joints velocity state model simulation'))
plot2Arrays(MAE(0.001*np.transpose(states[0:7,:]),30),q[0:30000:50,:],'state','true',\
    'Joints Position State Model Simulation')

"""

# Show all figures
plt.show()

 
































""" 
tau = kinova.computeTrajectoryTorques(q,qp,qpp,-0.11*np.ones((q.shape[0],q.shape[1],6)))
plot2Arrays(data.torque_rne,tau,"blast","Pinocchoi","Blast RNEA vs Pinnochoi RNEA ")
plot2Arrays(fildata['torque'],tau,"Sensor","Pinocchoi","Sensor vs Pinnochoi RNEA ")
    def kalmanFilter(self,variable='torque'):
        Filtering Robot Data torques uisng an adaptive kalman filter
        if variable =='torque_cur':
            observations = np.transpose(self.torque_cur)
            x0 = self.torque_cur[0,:]
        elif variable == 'torque_rne':
            observations = np.transpose(self.torque_rne)
            x0 = self.torque_rne[0,:]
        elif variable =='torque':
            observations = np.transpose(self.torque)
            x0 = self.torque[0,:]
        else:
            logger.error('variable type not supported yet')
        F = np.eye(self.ndof)
        H = np.random.randn(self.ndof, self.ndof)
        Q = np.eye(self.ndof) * 0.01   
        R = np.eye(self.ndof) * 0.1   
        P = np.eye(self.ndof)   
        kf = Kalman(F, H, Q, R, P, x0)
        estimated_states, innovations = kf.filter(observations)
        
        return estimated_states, innovations
"""

 


 
