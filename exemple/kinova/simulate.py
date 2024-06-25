import sys
import os
import matplotlib.pyplot as plt 
import numpy as np 

src_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'src'))
sys.path.append(src_folder)

from dynamics import Robot
from utils import RobotData, plotArray, plot2Arrays

data = RobotData(\
"C:/Users/chiha/OneDrive/Bureau/Dynamium/dynamic-identification/data/kinova/identification_data/RunTrajectory1.csv")
fildata = data.LowPassfilter(10)
kinova= Robot()
# check the configuration vector q of pinnochoi 
print(np.shape(kinova.q))
 
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

"""  



plotArray(tau,"model torques")
plotArray(fildata['torque'],"recodred torques")
plotArrays(tau,fildata['torque'],'simulation','recorded')
plt.show()
print(tau.shape)
print(type(qp))
 """
 