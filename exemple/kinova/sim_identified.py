import numpy as np
import matplotlib.pyplot as plt 
import identification_utils as iu


figureFolderPath = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/figure/kinova"
initial_guess_path = "C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/autogen/initial_guess_nlopt_best_torque_current.npy"   

x_opt = np.load(initial_guess_path)
print("Loaded initial guess from file.")

iu.validation(x_opt)
plt.show()