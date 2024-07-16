import os
import numpy as np
from scipy.optimize import least_squares
import identification_utils as iu


initial_guess_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/initial_guess_least_squares.npy"
dim = 13 * 7 + 6   

# Check if initial guess exists, otherwise create random initial guess
if os.path.exists(initial_guess_path):
    initial_guess = np.load(initial_guess_path)
    print("Loaded initial guess from file.")
else:
    initial_guess = np.random.rand(dim)
    print("Using random initial guess.")

# Run least squares optimization
result = least_squares(iu.objective_function2, initial_guess,verbose=1,max_nfev=3, method='dogbox')
# Save the optimized parameters
np.save(initial_guess_path, result.x)
print("Saved optimized parameters to file.")

optimal_params = result.x
minimized_rmse = np.sqrt(np.mean(result.fun**2))  # RMSE calculation

print(f"Optimal parameters: {optimal_params}")
print(f"Minimized RMSE: {minimized_rmse}")

# Visualization (assuming plot2Arrays function is defined in identification_utils)
