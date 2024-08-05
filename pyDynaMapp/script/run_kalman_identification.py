"""
kalman filter based identification of the robot state space system 
estimate : the default sytem --> velocity and poistion x(t)
estimate the augmented system state -> z(t) = [x(t) τ(t)]-> estimate the torque 

use the robust kalman for the fully state space estimation of evlocity ,
y=use the robust to the augmented 
use the simple filter kalmn for the linear model 
use the simple kalmn filter for the linear aumented model 

Note: 
P0  (Initial State Covariance): This matrix represents the initial uncertainty in the state estimate.
A higher value indicates more initial uncertainty.

Q (Process Noise Covariance): This matrix represents the uncertainty in the process model.
A higher value indicates that you trust the model less and the measurements more.

R (Measurement Noise Covariance): This matrix represents the uncertainty in the measurements. 
A higher value indicates that you trust the measurements less and the model more.
"""
import os
import sys
import argparse 
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from identification import Kalman, RobustKalman
from dynamics import StateSpace, Robot
from utils import RobotData, plot2Arrays, plotArray, yaml2dict, MAE

base_dir = os.getcwd()
figure_path      = os.path.join(base_dir ,"pyDynaMapp/figure/kinova") 
config_file_path = os.path.join(base_dir,"pyDynaMapp/robot/kinova/config.yml")
state_poles_path = os.path.join(base_dir,"pyDynaMapp/autogen/state_poles.npy")  
data_file_path   = os.path.join(base_dir,"pyDynaMapp/data/kinova/identification_data/blast_traj.csv")
urdf_file_path   = os.path.join(base_dir,"pyDynaMapp/robot/kinova/gen3.urdf")

parser = argparse.ArgumentParser()
parser.add_argument('--v',type=bool,default=False)
parser.add_argument('--cutoff_frequency', type=float, default=3)
parser.add_argument('--show_figures', type=bool,default=False)
args = parser.parse_args()

cutoff_frequency = args.cutoff_frequency
config_params  = yaml2dict(config_file_path)
data           = RobotData(data_file_path)
q              = data.position
qp             = data.velocity
qpp            = data.desiredAcceleration
torque_cur     = data.torque_cur

fildata        = data.lowPassfilter(cutoff_frequency)
q_f            = fildata['position']
qp_f           = fildata['velocity']
qpp_f          = fildata['desiredAcceleration']
torque_f       = fildata['torque']
torque_cur_f   = fildata['torque_cur']

kinova         = Robot(urdf_file_path,config_file_path)
kinova_ss      = StateSpace(urdf_file_path,config_file_path)

ndof = kinova.model.nq
num_samples = data.numRows

A = np.zeros((2*ndof,2*ndof*num_samples))
B = np.zeros((2*ndof,ndof*num_samples))
H = np.zeros((2*ndof,2*ndof*num_samples))
P0 = np.zeros((2*ndof,2*ndof*num_samples))
R  = np.zeros((2*ndof,num_samples))
Q = np.zeros((2*ndof,2*ndof*num_samples))


start = 100
step = 1
stop = 5000
for i in range(start, stop,step):
    print(f'process sample = {i}/{num_samples}')
    Ai, Bi, _, _ = kinova_ss.computeStateMatrices(q_f[i, :], qp_f[i, :])
    
    if np.any(np.isnan(Ai)) or np.any(np.isnan(Bi)) or np.any(np.isinf(Ai)) or np.any(np.isinf(Bi)):
        print(f"Warning: NaN or Inf detected in Ai or Bi at sample {i}")
        Ai = np.nan_to_num(Ai)  
        Bi = np.nan_to_num(Bi)
    
    if np.any(np.abs(Ai) > 1e7) or np.any(np.abs(Bi) > 1e7):
        print(f"Warning: Large values detected in Ai or Bi at sample {i}")
    
    A[:, (i-1)*2*ndof:i*2*ndof] = Ai
    B[:, (i-1)*ndof:i*ndof] = Bi
    H[:, (i-1)*2*ndof:i*2*ndof] =  np.eye(2*ndof)
    Q[:, (i-1)*2*ndof:i*2*ndof] = np.diag([.5] * 2*ndof)   
    R[:, i] = 0.2* np.ones(2*ndof)   
    P0[:, (i-1)*2*ndof:i*2*ndof] = np.diag([0.01] * 2*ndof)
    

x0 = kinova_ss.getStateVector(qp[start,:],q[start,:])                            
sim_states = np.zeros((2*ndof,num_samples))
kf = RobustKalman(A, B, H, Q, R, P0, x0)
for k in range(start, stop,step):
    u = 0.1*torque_f[k,:]
    z = kinova_ss.getStateVector(qp_f[k,:],q_f[k,:])
    x, _ = kf.step(u, z, k)
    sim_states[:,k] = x
    print(f"Time {k}:, state RMSE {np.sqrt(np.mean((x-z)**2))}")
    
states = sim_states[7:14, start:stop:step]
vel = qp[start:stop:step,:]
scaled_states = 2*(states - np.min(states))/(np.max(states)-np.min(states))
scaled_vel = 2*(vel - np.min(vel))/(np.max(vel)-np.min(vel))
plot2Arrays(scaled_states.T,scaled_vel,'state','true',\
f'Joints Velocity State Model Simulation, cutoff = {cutoff_frequency}, sampling ={data.samplingRate}')
plt.show()
    
    




