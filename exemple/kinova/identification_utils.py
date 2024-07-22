####################################################################################
#
#
#
# 
# 
####################################################################################
import sys
import os
import seaborn as sns
import matplotlib.pyplot as plt 
import numpy as np 
import logging

figureFolderPath = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/figure/kinova"
config_file_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml"

src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '../src'))
sys.path.append(src_folder)

if not os.path.exists(figureFolderPath):
    os.makedirs(figureFolderPath)

from dynamics import Robot, Regressor, StateSpace
from utils import RobotData, plot2Arrays, yaml2dict, RMSE, MAE

mlogger  = logging.getLogger('matplotlib')
logging.basicConfig(level='INFO')
mlogger.setLevel(logging.WARNING)

dynamics_logger = logging.getLogger('dynamics')
dynamics_logger.setLevel(logging.ERROR)

cutoff_frequency  = 3
config_params  = yaml2dict(config_file_path)
data           = RobotData(config_params['identification']['dataFilePath'])
fildata        = data.lowPassfilter(cutoff_frequency)
kinova         = Robot()
q_f            = fildata['position']
qp_f           = fildata['velocity']
qpp_f          = fildata['desiredAcceleration']
current_f      = fildata['current']
torque_f       = fildata['torque']
torque_cur_f   = fildata['torque_cur'] 

q              = data.position
qp             = data.velocity
qpp            = data.desiredAcceleration
current        = data.current
torque         = data.torque

iteration_counter = 0

def objective_function1(x, grad): 
    global kinova, iteration_counter, q_f, qp_f, qpp_f, torque_f
    kinova.setIdentificationModelData(q_f, qp_f, qpp_f)
    tau_sim = kinova.computeIdentificationModel(x)
    rmse_time  = RMSE(torque_f, tau_sim, axis=1)
     
    print(
        f"Iteration {iteration_counter}: "
        f"RMSE = {np.sqrt(np.mean(rmse_time**2)):.5f}"
    )
    iteration_counter += 1
    return np.sqrt(np.mean(rmse_time**2))

def objective_function2(x, grad): 
    global kinova, iteration_counter, q_f, qp_f, qpp_f, torque_f
    kinova.setIdentificationModelData(q_f, qp_f, qpp_f)
    tau_sim = kinova.computeIdentificationModel(x)
    rmse_time  = RMSE(torque_cur_f, tau_sim, axis=1)
     
    print(
        f"Iteration {iteration_counter}: "
        f"RMSE = {np.sqrt(np.mean(rmse_time**2)):.5f}"
    )
    iteration_counter += 1
    return np.sqrt(np.mean(rmse_time**2))
 
def validation(x):
    global kinova, q_f, qp_f, qpp_f, torque_f, figureFolderPath
    kinova.setIdentificationModelData(q_f,qp_f,qpp_f)
    tau_sim = kinova.computeIdentificationModel(x)
    rmse_time  = RMSE(torque_f, tau_sim, axis=1)
    r = np.sqrt(np.mean(rmse_time**2))
    # rescale the torques values by the max values from datasehht that can the  
    # save the values of the torque simulted or comuted from the model to csv file 
    format = {'fmt': '%.4f', 'delimiter': ', ', 'newline': ',\n'}
    np.savetxt('C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/model_simulation_torques.csv', tau_sim, **format)
    
    plot2Arrays(torque_f,tau_sim,"true","simulation",f"Manipulator Optimized Non Linear model NLopt-MaxNelder RMSE ={r}")
    plt.savefig(os.path.join(figureFolderPath,'non_Linear_model_nlopt_best_poly'))
    plt.show()














def objective_function2(x, gard):
    global kinova, iteration_counter, q_f, qp_f, qpp_f, torque_f, data, torque
    kinova.updateInertiaParams(x[0:13*7])
    kinova.updateExternalForces(x[13*7:13*7+6])
    tau_sim = np.zeros_like(torque)
    for i  in range(data.numRows):
        tau_sim[i,:] = kinova.computeDifferentialModel(q_f[i,:],qp_f[i,:],qpp_f[i,:])
    rmse_time  = RMSE(torque_f, tau_sim, axis=1)
    print(
        f"Evaluation {iteration_counter}: "
        f"RMSE = {np.sqrt(np.mean(rmse_time**2)):.5f}"
    )
    iteration_counter += 1
    return np.sqrt(np.mean(rmse_time**2))