import numpy as np 
import seaborn as sns
from matplotlib import pyplot as plt

class Trajectory():
    """
    Base class for general tarjectory motion generation.
    Args:
        ndof - robot degree of freedom
        sampling - sampling time-genration frequancy
        nbWaypoints - number of genated pointed of the trakejctory  
    Ref:
    
    """
    def __init__(self,ndof=7,sampling = 1000,ti=0,tf=1000) -> None:
        self.ndof = ndof
        self.samplingFrequency = sampling
        self.ti = 0 
        self.tf = 1000
        
    def setTrajectoryData(self, time, Q, Qp, Qpp)->None:
        """
        
        """
        mq,nq = Q.shape
        mqp,nqp = Qp.shape
        mqpp,nqpp = Qpp.shape
        if (mq != mqp) or (mqp != mqpp) or (mq != mqpp) or \
            (nq != nqp) or (nqp != nqpp) or (nq != nqpp):
            raise ValueError('Trajectory engine: incorrect data dimensions!')
    
    def getTrajectoryData(self):
        """
        
        """
        
        return 
    
    
    def plotTrajectory(self)->None:
        """
        
        """
        sns.set(style="whitegrid")
        fig, axes = plt.subplots(3, 3, figsize=(10, 6))
        for i in range(7):
            ax = axes[i // 3, i % 3]
            sns.lineplot(ax=ax, x=np.arange(len(self.current[:, i])),\
                y=self.current[:, i],linewidth=0.5)
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Current (mA)")
            ax.set_title(f'Joint {i+1}')
        fig.suptitle('Joints Trajectory', fontsize=11)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()