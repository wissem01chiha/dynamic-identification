import argparse
import sys
import os
import numpy as np 
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt 
import logging
import time

logger = logging.getLogger(__name__)
st = time.time()

parser = argparse.ArgumentParser(description=\
'simulate manipulator state space models with default paramters')

parser.add_argument('--v',type=bool,default=False)
parser.add_argument('--cutoff_frequency', type=float, default=3)
parser.add_argument('--show_figures', type=bool,default=False)
parser.add_argument('--data_file',type=str,default='blast_traj.csv')
parser.add_argument('--filter',type=bool,default=True)
args = parser.parse_args()

base_dir = os. getcwd()
pkg_dir = os.path.join(base_dir,'pyDynaMapp')
sys.path.append(pkg_dir)

figure_path = os.path.join(pkg_dir ,"figure/kinova") 
config_file_path = os.path.join(pkg_dir,"robot/kinova/config.yml")
state_poles_path = os.path.join(pkg_dir,"autogen/state_poles.npy")  

if not os.path.exists(figure_path):
    os.makedirs(figure_path)

from dynamics.robot import Robot
from dynamics.state_space import StateSpace
from utils import RobotData, plot2Arrays, yaml2dict, MAE

if not(args.v):
    dynamics_logger = logging.getLogger('dynamics')
    dynamics_logger.setLevel(logging.ERROR)

cutoff_frequency = args.cutoff_frequency
config_params  = yaml2dict(config_file_path)
data           = RobotData(config_params['identification']['dataFilePath'])
fildata        = data.lowPassfilter(cutoff_frequency)
kinova         = Robot()
q_f            = fildata['position']
qp_f           = fildata['velocity']
qpp_f          = fildata['desiredAcceleration']
current_f      = fildata['current']
torque_f       = fildata['torque']
q              = data.position
qp             = data.velocity
qpp            = data.desiredAcceleration
current        = data.current
torque         = data.torque
 
kinova_ss = StateSpace(kinova)
tau_ss    = torque
x0        = kinova_ss.getStateVector(qp_f[0,:],q_f[0,:])
start     = 0
step      = 100
end       = 30000

states = kinova_ss.simulate(x0,tau_ss[start:end:step,:],verbose=True)
nyquist_freq = 0.5 * data.samplingRate

if args.filter:
    normal_cutoff = cutoff_frequency / nyquist_freq
    b, a = butter(1, normal_cutoff, btype='low', analog=False)
    #states =  filtfilt(b,a,states,axis=1)

plot2Arrays(4*MAE(np.transpose(np.clip(states[7:14,:],np.min(qp_f),np.max(qp_f))),10),qp_f[start:end:step,:],'state','true',\
f'Joints Velocity State Model Simulation, window = {step}, cutoff = {cutoff_frequency}, sampling ={data.samplingRate}')
plt.savefig(os.path.join(figure_path,'joints velocity state model simulation'))
if args.v :
    et = time.time()-st
    logger.info(f'elapsed time : {et}')

if args.show_figures:
    plt.show()
