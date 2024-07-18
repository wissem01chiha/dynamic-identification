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
    
from dynamics import Robot, Regressor
from trajectory import FourierGenerator
from utils import RobotData,  plot2Arrays, plotElementWiseArray, yaml2dict, RMSE, MAE


# TODO : plot the evolution par time of the trajectory identification criteria J 
# and the RMSE evolution 




config_params  = yaml2dict(config_file_path)
traj = FourierGenerator(config_params['trajectory'])
tspan = 1
Q, Qp, Qpp = traj.computeFullTrajectory(0,tspan,q0,qp0,qpp0)
