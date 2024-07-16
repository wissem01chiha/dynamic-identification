import os  
import numpy as np
import nlopt 
import identification_utils as iu

initial_guess_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/initial_guess_scipy.npy"  # File to save and load the initial guess


#####################################################################################
# optimisation routines 
#####################################################################################
# Initialize the optimizer
dim = 13*7+6  # Dimension of the input vector
max_iter = 60
opt = nlopt.opt(nlopt.LN_NELDERMEAD, dim)  # Example optimizer (choose an appropriate one)
# Set the objective function
opt.set_min_objective(iu.objective_function2)
# Set optimization parameters (optional)
opt.set_maxeval(max_iter)  # Maximum number of evaluations
opt.set_ftol_rel(1e-5)     # Relative tolerance on function value
opt.set_xtol_rel(1e-5)
# Define bounds if necessary (optional)
lower_bounds = np.full(dim, -5)
upper_bounds = np.full(dim, 5)
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
# Save the optimized vector for future use 
np.save(initial_guess_path, x_opt)
print("Saved optimized parameters to file.")
##############################################################################
# visulization
##############################################################################

# pour les paramters optimizées trouver le meilleur trajectoire identifable 
# genrer beaucoup des trajs ---- Fourier , enregister leur params, ( coeffs, criteria 
# cond(W) )
# sur chaqun excuetr l optimisation ---> error RMSE model-reel ,
# save the final RMSE of each traj 
# TODO : plot : 2D plot of the RMSe (y) en fontion du frequance du traj (FHz)
# TODO : plot :  2D plot of the RMSE (y) en fonction du sampling rate du traj (Fs)
# TODO : plot : 3D plot of the RMSE en fonction du FHz and Fs 
# TODO : for the linear model : plot of the RMSE with the Cond(W) or the crteria choosen 