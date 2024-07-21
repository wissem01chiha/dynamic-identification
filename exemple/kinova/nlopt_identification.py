import os  
import numpy as np
import nlopt 
import identification_utils as iu

initial_guess_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/initial_guess_nlopt_best_torque_sensor.npy"  # File to save and load the initial guess

#####################################################################################
# optimisation routines 
#####################################################################################
# Initialize the optimizer
dim = 209  # Dimension of the input vector
max_iter = 4
opt = nlopt.opt(nlopt.LN_NELDERMEAD, dim) 
# Set the objective function
opt.set_min_objective(iu.objective_function1)
# Set optimization parameters (optional)
opt.set_maxeval(max_iter)  # Maximum number of evaluations
opt.set_ftol_rel(1e-6)     # Relative tolerance on function value
opt.set_xtol_rel(1e-6)
# Define bounds if necessary (optional)
lower_bounds = np.full(dim,-100)
upper_bounds = np.full(dim, 100)
opt.set_lower_bounds(lower_bounds)
opt.set_upper_bounds(upper_bounds)
# Initial guess for the optimization
if os.path.exists(initial_guess_path):
    initial_guess = np.load(initial_guess_path)
    print("Loaded initial guess from file.")
else:
    initial_guess = np.random.rand(dim)
    print("Using random initial guess.")
# Run the optimization
x_opt = opt.optimize(initial_guess)
min_value = opt.last_optimum_value()
result_code = opt.last_optimize_result()
print(f'paramters values : {x_opt}')
print(f'minimum value de la fonction objective: {min_value}')

# Save the optimized vector for future use 
np.save(initial_guess_path, x_opt)
print("Saved optimized parameters to file.")
##############################################################################
# visulization
##############################################################################


iu.validation(x_opt)
