#############################################################################################
# pour les paramters optimizées trouver le meilleur trajectoire identifable 
# genrer beaucoup des trajs ---- Fourier , enregister leur params, ( coeffs, criteria 
# cond(W) )
# sur chaqun excuetr l optimisation ---> error RMSE model-reel ,
# save the final RMSE of each traj 
# TODO : plot : 2D plot of the RMSe (y) en fontion du frequance du traj (FHz)
# TODO : plot :  2D plot of the RMSE (y) en fonction du sampling rate du traj (Fs)
# TODO : plot : 3D plot of the RMSE en fonction du FHz and Fs 
# TODO : for the linear model : plot of the RMSE with the Cond(W) or the crteria choosen
# NOTE: to compute the RMSE of a traj we nedd the real data of the robot suiving the traj  
##############################################################################################
import sys
import os
import logging
import nlopt 
import matplotlib.pyplot as plt 
import numpy as np 
 
traj_parms_path = "/home/wissem/dynamic-identification/autogen/traj_paramters.npy"
figureFolderPath="/home/wissem/dynamic-identification/figure/kinova"
config_file_path="/home/wissem/dynamic-identification/exemple/kinova/config.yml"
params_file_path = "/home/wissem/dynamic-identification/autogen/initial_guess_nlopt_best_torque_current.npy"   
src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)),'../src'))
sys.path.append(src_folder)
if not os.path.exists(figureFolderPath):
    os.makedirs(figureFolderPath)
    
from dynamics import Robot, Regressor
from trajectory import FourierGenerator
from utils import RobotData,  plot2Arrays, plotElementWiseArray, yaml2dict, RMSE, MAE

dynamics_logger = logging.getLogger('dynamics')
dynamics_logger.setLevel(logging.ERROR)

# TODO : plot the evolution par time of the trajectory identification criteria J 
# and the RMSE evolution
dim = 209 


if os.path.exists(params_file_path):
    x = np.load(params_file_path)
    print("Loaded initial params guess from file.")
else:
    x= np.random.rand(dim)
    print("Using random initial guess.")

config_params  = yaml2dict(config_file_path)
traj = FourierGenerator(config_params['trajectory'])
tspan = (traj.trajectory_params['samples'] +1)* 0.001
q0 = np.zeros(7)
qp0 =np.zeros(7)
qpp0 = np.zeros(7)
Q, Qp, Qpp = traj.computeFullTrajectory(0,tspan,q0,qp0,qpp0)
iteration_counter = 0
  
def computeTrajectoryError(traj_parms,grad):
    """this function  compute the best forier paramters (ie trajectory) on wich the differentiation 
    error du non linear torques function est min , this is designed to work with nlopt
    traj_parms = 7*10 = 70 
    """
    global q0, qp0, qpp0, tspan, config_params, iteration_counter
    traj = FourierGenerator(config_params['trajectory'])
    traj.trajectory_params['frequancy'] = 1
    traj.trajectory_params['Aij'] = np.reshape(traj_parms[1:36],(-1,5))
    traj.trajectory_params['Bij'] = np.reshape(traj_parms[36:71],(-1,5))
    err = traj.computeConstraintedDifferentiationError(0,tspan,x,q0,qp0,qpp0)
    print(
        f"Iteration {iteration_counter}: "
        f"RMSE = {err:.5f}"
    )
    iteration_counter +=1
    return err

###########################################################################
max_iter = 20
opt = nlopt.opt(nlopt.LN_NELDERMEAD, 71) 
opt.set_min_objective(computeTrajectoryError)
opt.set_maxeval(max_iter)  # Maximum number of evaluations
opt.set_ftol_rel(1e-6)     # Relative tolerance on function value
opt.set_xtol_rel(1e-6)
lower_bounds = np.full(71,-100)
upper_bounds = np.full(71, 100)
#opt.set_lower_bounds(lower_bounds)
#opt.set_upper_bounds(upper_bounds)

if os.path.exists(traj_parms_path):
    initial_guess = np.load(traj_parms_path)
    print("Loaded initial guess from file.")
    
x_opt = opt.optimize(initial_guess)
min_value = opt.last_optimum_value()
result_code = opt.last_optimize_result()
print(f'paramters values : {x_opt}')
print(f'minimum value de la fonction objective: {min_value}')
np.save(traj_parms_path, x_opt)
print("Saved optimized parameters to file.")

# vis the computed traj 
best_traj_parms = np.load(traj_parms_path)
traj = FourierGenerator(config_params['trajectory'])
traj.trajectory_params['frequancy'] = 1
traj.trajectory_params['Aij'] = np.reshape(best_traj_parms[1:36],(-1,5))
traj.trajectory_params['Bij'] = np.reshape(best_traj_parms[36:71],(-1,5))

traj.visualizeTrajectory(0,tspan,q0,qp0,qpp0,figureFolderPath)
q, qp, qpp = traj.computeFullTrajectory(0,tspan,q0,qp0,qpp0)
np.savetxt('/home/wissem/dynamic-identification/autogen/computed_fourier_traj_position.csv',q)
np.savetxt('/home/wissem/dynamic-identification/autogen/computed_fourier_traj_velocity.csv',qp)
np.savetxt('/home/wissem/dynamic-identification/autogen/computed_fourier_traj_acceleration.csv',qpp)
plt.show()
