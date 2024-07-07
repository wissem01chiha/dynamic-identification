""" 

"""
import sys
import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt 
import numpy as np 
import logging
import time

logging.basicConfig(level='INFO')

mlogger = logging.getLogger('matplotlib')
mlogger.setLevel(logging.WARNING)

src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Robot, StateSpace
from utils import RobotData, plotArray, plot2Arrays, yaml2dict, RMSE, MAE

cutoff_frequency = 3
config_params    = yaml2dict("C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml")
data             = RobotData(config_params['identification']['dataFilePath'])

fildata = data.LowPassfilter(cutoff_frequency)
kinova = Robot()
 
q_f       = fildata['position']
qp_f      = fildata['velocity']
qpp_f     = fildata['desiredAcceleration']
current_f = fildata['current']
torque_f  = fildata['torque']

q   = data.position
qp  = data.velocity
qpp = data.desiredAcceleration
current = data.current
torque = data.torque

rmse_joint = RMSE(torque, data.torque_rne).flatten()
rmse_time  = RMSE(torque, data.torque_rne,axis=1) 
# compute the model with computed matrices tau =M(q)qddot+C(q,qp)qp+G(q) 
tau_sim = np.zeros_like(torque)
x = 0.0001*np.abs(np.random.rand(13*7))
tau_f =kinova.computeFrictionTorques(qp,q)
for i  in range(data.numRows):
    tau_sim[i,:] = 3*(kinova.computeDifferentialModel(q[i,:],qp[i,:],qpp[i,:],x) +tau_f[i,:])
    
plot2Arrays(torque_f,tau_sim,"true","sim","diff model sec")
plt.show()
"""  
# -----------------------------------------------------------------------------------------
# simulate the state space model 
kinova_ss = StateSpace(kinova)
tau_ss = torque
x0 = kinova_ss.getStateVector(qp[0,:],q[0,:])
states = kinova_ss.simulate(x0,tau_ss[:30000:50,:])
plot2Arrays(MAE(0.001*np.transpose(states[7:14,:]),30),qp[:30000:50,:],'state','true',\
    'Joints Velocity State Model Simulation')
plot2Arrays(MAE(0.001*np.transpose(states[0:7,:]),30),q[0:30000:50,:],'state','true',\
    'Joints Position State Model Simulation')
#----------------------------------------------------------------------------------------
df = pd.DataFrame({'Index':np.ones_like(range(len(rmse_joint)))+range(len(rmse_joint)), 'Value': rmse_joint})
plt.figure(figsize=(10,6))
sns.barplot(x= np.ones_like(range(len(rmse_joint)))+range(len(rmse_joint)), y=rmse_joint)
plt.xlabel('Joint Index',fontsize=9)
plt.ylabel('RMSE',fontsize=9)
plt.title('Error between Blast RNEA and Sensor Torques Per Joint',fontsize=9)

plotArray(rmse_time,'Error between Blast RNEA and Sensor Torques.','Error Value')
plot2Arrays(q, q_f, "true", "filtred", f"Joints Positions, cutoff frequency = {cutoff_frequency} Hz")
plot2Arrays(qp, qp_f, "true", "filtred", f"Joints Velocity, cutoff frequency = {cutoff_frequency} Hz")
plot2Arrays(qpp, qpp_f, "true", "filtred", f"Joints Acceleration, cutoff frequency = {cutoff_frequency} Hz")
plot2Arrays(torque, torque_f , "true", "filtred", f"Joints Torques, cutoff frequency = {cutoff_frequency} Hz")
plot2Arrays(current, current_f , "true", "filtred", f"Joints Current, cutoff frequency = {cutoff_frequency} Hz")
plt.show()

tau = kinova.computeTrajectoryTorques(q,qp,qpp,-0.11*np.ones((q.shape[0],q.shape[1],6)))
plot2Arrays(data.torque_rne,tau,"blast","Pinocchoi","Blast RNEA vs Pinnochoi RNEA ")
plot2Arrays(fildata['torque'],tau,"Sensor","Pinocchoi","Sensor vs Pinnochoi RNEA ")
"""

 


 
