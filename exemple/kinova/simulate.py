"""
 this script is used to 
    - plot the robot recored data
    - compute and plot the error(RMSE) and covarience parmters 
    - 
"""
# $env:PYTHONDONTWRITEBYTECODE=1   
import sys
import os
import matplotlib.pyplot as plt 
import numpy as np 

src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Robot
from utils import RobotData, plotArray, plot2Arrays, yaml2dict

config_params= yaml2dict("C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/exemple/kinova/config.yml")
data = RobotData(config_params['identification']['dataFilePath'])
fildata = data.LowPassfilter(8)
kinova= Robot()
 
q_f   = fildata['position']
qp_f  = fildata['velocity']
qpp_f = fildata['desiredAcceleration']
q   = data.position
qp  = data.velocity
qpp = data.desiredAcceleration
tau = kinova.computeTrajectoryTorques(q,qp,qpp,-0.11*np.ones((q.shape[0],q.shape[1],6)))
 
 
plt.close('all')
plotArray(q_f,"Filtred Joints position")
 
plotArray(qp_f,"Filtred Joints velocity")
plotArray(qpp_f,"Filtred Joints acceleration")
plot2Arrays(data.torque_rne,data.torque,"RNEA","Sensor","Blast RNEA vs Sensor")
plot2Arrays(data.torque_rne,tau,"blast","Pinocchoi","Blast RNEA vs Pinnochoi RNEA ")
plot2Arrays(fildata['torque'],tau,"Sensor","Pinocchoi","Sensor vs Pinnochoi RNEA ")
plt.show()

