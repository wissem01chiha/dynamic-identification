import sys
import os
import seaborn as sns
import matplotlib.pyplot as plt 
import numpy as np 
import logging
import time
import nlopt 

# Define paths
figureFolderPath = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/figure/kinova"
config_file_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml"
initial_guess_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/initial_guess.npy"  # File to save and load the initial guess

src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '../src'))
sys.path.append(src_folder)

if not os.path.exists(figureFolderPath):
    os.makedirs(figureFolderPath)

from dynamics import Robot, Regressor, StateSpace
from utils import RobotData, plot2Arrays, plotElementWiseArray, yaml2dict, RMSE, MAE


mlogger  = logging.getLogger('matplotlib')
logging.basicConfig(level='INFO')
mlogger.setLevel(logging.WARNING)

# Suppress warnings from the Robot class or dynamics module
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

q              = data.position
qp             = data.velocity
qpp            = data.desiredAcceleration
current        = data.current
torque         = data.torque

iteration_counter = 0

def objective_function(x, grad): 
    global iteration_counter
    config_file_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml"
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
    kinova.setIdentificationModelData(q_f, qp_f, qpp_f)
    tau_sim = kinova.computeIdentificationModel(x)
    rmse_time  = RMSE(torque_f, tau_sim, axis=1)
     
    print(
        f"Iteration {iteration_counter}: "
        f"RMSE = {np.sqrt(np.mean(rmse_time**2)):.5f}"
    )
    iteration_counter += 1
    return np.sqrt(np.mean(rmse_time**2))


# Initialize the optimizer
dim = 209  # Dimension of the input vector
opt = nlopt.opt(nlopt.LN_COBYLA, dim)  # Example optimizer (choose an appropriate one)

# Set the objective function
opt.set_min_objective(objective_function)

# Set optimization parameters (optional)
opt.set_maxeval(1000)  # Maximum number of evaluations
opt.set_ftol_rel(1e-4)  # Relative tolerance on function value
opt.set_xtol_rel(1e-4)

# Define bounds if necessary (optional)
lower_bounds = np.full(dim, -10)
upper_bounds = np.full(dim, 10)
#opt.set_lower_bounds(lower_bounds)
#opt.set_upper_bounds(upper_bounds)

# Initial guess for the optimization
if os.path.exists(initial_guess_path):
    initial_guess = np.load(initial_guess_path)
    print("Loaded initial guess from file.")
else:
    initial_guess = np.random.rand(dim)
    print("Using random initial guess.")

# Run the optimization
x_opt = opt.optimize(np.array(initial_guess))
min_value = opt.last_optimum_value()
result_code = opt.last_optimize_result()

# Save the optimized vector for future use
np.save(initial_guess_path, x_opt)
print("Saved optimized parameters to file.")

kinova.setIdentificationModelData(q_f, qp_f, qpp_f)
tau_sim = kinova.computeIdentificationModel(x_opt)
tau_sim[0:15,6]=0
plot2Arrays(torque_f, tau_sim, "true", "simulation",\
f"Manipulator Optimised Non Linear Model: Nlopt-NELDERMEAD, RMSE = {min_value:.4f}, Fs = {data.samplingRate:.4f}, Fc ={cutoff_frequency:.4f} ")
plt.show()

# Output the results
print("Optimized parameters:", x_opt)
print("Minimum value of the objective function:", min_value)
print("Optimization result code:", result_code)

# pour les paramters optimizées trouver le meilleur trajectoire identifable 
# genrer beaucoup des trajs ---- Fourier , enregister leur params, ( coeffs, criteria 
# cond(W) )
# sur chaqun excuetr l optimisation ---> error RMSE model-reel ,
# save the final RMSE of each traj 
# TODO : plot : 2D plot of the RMSe (y) en fontion du frequance du traj (FHz)
# TODO : plot :  2D plot of the RMSE (y) en fonction du sampling rate du traj (Fs)
# TODO : plot : 3D plot of the RMSE en fonction du FHz and Fs 
# TODO : for the linear model : plot of the RMSE with the Cond(W) or the crteria choosen 
# 
