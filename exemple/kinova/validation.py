# this script used to validate the results on the other trajectories 
# of the data sampled 
import sys
import os
import seaborn as sns
import matplotlib.pyplot as plt 
import numpy as np 
import logging
 

figureFolderPath="C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/figure/kinova"
config_file_path="C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml"
src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)),'../src'))
sys.path.append(src_folder)
if not os.path.exists(figureFolderPath):
    os.makedirs(figureFolderPath)

from dynamics import Robot, Regressor, StateSpace
from utils import RobotData,  plot3Arrays, yaml2dict

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

params_file_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/autogen/initial_guess_nlopt_best.npy"   
if os.path.exists(params_file_path):
    params = np.load(params_file_path)
    print("Loaded paramters values from file.")
else:
    mlogger.error('params file path not found !')
kinova.setIdentificationModelData(q_f,qp_f,qpp_f)
tau_sim = kinova.computeIdentificationModel(params)   
plot3Arrays(torque_f, tau_sim,data.torque_rne,'filtred sensor','simulation','blast')
plt.show()