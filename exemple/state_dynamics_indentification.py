import argparse
import sys
import os
import matplotlib.pyplot as plt 
import numpy as np 
import nlopt
import logging
from scipy.signal import butter, filtfilt

parser = argparse.ArgumentParser(description=\
'simulate manipulator state space models with default paramters')

parser.add_argument('--v',type=bool)
parser.add_argument('--cutoff_frequency', type=float, default=3)
parser.add_argument('--show_figures', type=bool,default=False)
parser.add_argument('--data_file',type=str,default='b')
parser.add_argument('--iteration',type=int,default=10)
args = parser.parse_args()

base_dir = os. getcwd()
figure_path = os.path.join(base_dir,"figure/kinova") 
config_file_path = os.path.join(base_dir,"exemple/kinova/config.yml")
state_poles_path = os.path.join(base_dir,"autogen/state_poles.npy")  

src_folder = os.path.join(base_dir,"src")
sys.path.append(src_folder)
if not os.path.exists(figure_path):
    os.makedirs(figure_path)

from dynamics import Robot,  StateSpace
from utils import RobotData,  plot2Arrays, yaml2dict, RMSE

dynamics_logger = logging.getLogger('dynamics')
dynamics_logger.setLevel(logging.ERROR)

mlogger  = logging.getLogger('matplotlib')
logging.basicConfig(level='INFO')
mlogger.setLevel(logging.WARNING)

cutoff_frequency  = args.cutoff_frequency
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

kinova_ss = StateSpace(kinova)
tau_ss = torque
x0 = kinova_ss.getStateVector(qp_f[0,:],q_f[0,:])

start =0 
end = 1000
###################################################################
# define optimisation routines 
iteration_counter = 0
def optimize_poles(x,grad):
    global kinova_ss, tau_ss, x0, iteration_counter 
    kinova_ss.robot.params['state_space_params']['poles']= x
    states = kinova_ss.simulate(x0,tau_ss[start:end])
    rmse_time  = RMSE(np.transpose(states[7:14,:]), qp[start:end], axis=1)
    print(
        f"Iteration {iteration_counter}:"
        f"RMSE = {np.sqrt(np.mean(rmse_time**2)):.5f}"
    )
    iteration_counter +=1
    return np.sqrt(np.mean(rmse_time**2))
    
dim = 14   
max_iter = args.iteration

opt = nlopt.opt(nlopt.LN_NELDERMEAD, dim) 
opt.set_min_objective(optimize_poles)
opt.set_maxeval(max_iter)  
opt.set_ftol_rel(1e-6)     
opt.set_xtol_rel(1e-6)
# Define bounds if necessary (optional)
lower_bounds = np.full(dim,-1)
upper_bounds = np.full(dim, 0)
opt.set_lower_bounds(lower_bounds)
opt.set_upper_bounds(upper_bounds)
# Initial guess for the optimization
if os.path.exists(state_poles_path):
    initial_guess = np.load(state_poles_path)
    print("Loaded initial guess from file.")
else:
    initial_guess = -np.abs(np.random.rand(dim))
    print("Using random initial guess.")
    
x_opt = opt.optimize(initial_guess)
min_value = opt.last_optimum_value()
result_code = opt.last_optimize_result()
print(f'paramters values : {x_opt}')
print(f'minimum value de la fonction objective: {min_value}')

# Save the optimized vector for future use 
np.save(state_poles_path, x_opt)
print("Saved optimized parameters to file.")
####################################################################################
# validation 
kinova_ss.robot.params['state_space_params']['poles'] = x_opt
states = kinova_ss.simulate(x0,tau_ss[start:end,:],verbose=True)

nyquist_freq = 0.5 * 1000
normal_cutoff = cutoff_frequency / nyquist_freq
#b, a = butter(3, normal_cutoff, btype='low', analog=False)
#states =  filtfilt(b, a, states, axis=1)
print('finit filtring')

plot2Arrays(0.001*np.transpose(states[7:14,:]), qp[start:end,:],'state','true',\
    'Joints Velocity State Model Simulation')
plt.savefig(os.path.join(figure_path,'joints velocity state model simulation'))
#plot2Arrays(0.001*np.transpose(states[7:14,:]), q[:30000:200],'state','true',\
#   'Joints Position State Model Simulation')
if args.show_figures:
    plt.show()
