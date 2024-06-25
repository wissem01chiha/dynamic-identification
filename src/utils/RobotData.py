import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from   scipy.signal import butter, filtfilt

class RobotData:
    """
    Initialize the RobotData object by loading and processing data from a CSV file.
    
    Args:
        - dataFilePath (str): Path to the CSV data file.
        - intIndex (int): Initial index for data slicing.
        - stepIndex (int): Step index for periodic data selection.
        - fnlIndex (int): Final index for data slicing.
    """
    def __init__(self, dataFilePath, intIndex=None, stepIndex=1, fnlIndex=None):
        try:
            dataTable = pd.read_csv(dataFilePath)
        except Exception as e:
            raise ValueError(f"Error loading data: {e}")

        if intIndex is None:
            intIndex = 0
        if fnlIndex is None:
            fnlIndex = dataTable.shape[0]

        self.numRows = dataTable.shape[0]
        self.numCols = dataTable.shape[1]

        self.time = dataTable.iloc[intIndex:fnlIndex:stepIndex, 0].to_numpy()

        self.velocity = dataTable.iloc[intIndex:fnlIndex:stepIndex, 8:15].to_numpy()
        self.torque = dataTable.iloc[intIndex:fnlIndex:stepIndex, 22:28].to_numpy()      # torque_sm : torque recorded from torque sensor
        self.torque_cur = dataTable.iloc[intIndex:fnlIndex:stepIndex, 29:35].to_numpy()  # torque    : recorded using current
        self.torque_rne =  dataTable.iloc[intIndex:fnlIndex:stepIndex, 15:21].to_numpy() # torque    : from blast
        self.current = dataTable.iloc[intIndex:fnlIndex:stepIndex, 43:50].to_numpy()
        self.position = dataTable.iloc[intIndex:fnlIndex:stepIndex, 1:8].to_numpy()
        self.temperature = dataTable.iloc[intIndex:fnlIndex:stepIndex, 36:43].to_numpy()
        self.desiredVelocity = dataTable.iloc[intIndex:fnlIndex:stepIndex, 57:64].to_numpy()
        self.desiredPosition = dataTable.iloc[intIndex:fnlIndex:stepIndex, 50:57].to_numpy()
        self.desiredAcceleration = dataTable.iloc[intIndex:fnlIndex:stepIndex, 64:71].to_numpy()

        self.timeUnit = 'milliseconds'
        self.timeStep = 1e-3 
        self.samplingRate = 1000  
        self.duration = (len(self.time) - 1) * self.timeStep  

        self.maxJointVelocity = np.max(self.velocity, axis=0)
        self.maxJointTorque = np.max(self.torque, axis=0)
        self.maxJointCurrent = np.max(self.current, axis=0)
        self.maxJointTemperature = np.max(self.temperature, axis=0)
        self.maxJointPosition = np.max(self.position, axis=0)
        self.maxDesiredJointAcceleration = np.max(self.desiredAcceleration, axis=0)

        self.minJointVelocity = np.min(self.velocity, axis=0)
        self.minJointTorque = np.min(self.torque, axis=0)
        self.minJointCurrent = np.min(self.current, axis=0)
        self.minJointTemperature = np.min(self.temperature, axis=0)
        self.minJointPosition = np.min(self.position, axis=0)
        self.minDesiredJointAcceleration = np.min(self.desiredAcceleration, axis=0)

        self.corrJointPosition = np.corrcoef(self.position, rowvar=False)
        self.corrJointVelocity = np.corrcoef(self.velocity, rowvar=False)
        self.corrJointTorque   = np.corrcoef(self.torque, rowvar=False)
        self.corrJointCurrent  = np.corrcoef(self.current, rowvar=False)
        self.corrJointAcceleration = np.corrcoef(self.desiredAcceleration, rowvar=False)
        
    def updateSamplingRate(self, new_sampling_rate):
        self.samplingRate = new_sampling_rate
        self.timeStep = 1 / self.samplingRate
        self.duration = (len(self.time) - 1) * self.timeStep
        
    def LowPassfilter(self, cutoff_frequency):
        """ """
        nyquist_freq = 0.5 * self.samplingRate
        normal_cutoff = cutoff_frequency / nyquist_freq
        b, a = butter(4, normal_cutoff, btype='low', analog=False)
        smoothed_data = {}
        smoothed_data['velocity'] = filtfilt(b, a, self.velocity, axis=0)
        smoothed_data['torque'] = filtfilt(b, a, self.torque, axis=0)
        smoothed_data['current'] = filtfilt(b, a, self.current, axis=0)
        smoothed_data['position'] = filtfilt(b, a, self.position, axis=0)
        smoothed_data['temperature'] = filtfilt(b, a, self.temperature, axis=0)
        smoothed_data['desiredVelocity'] = filtfilt(b, a, self.desiredVelocity, axis=0)
        smoothed_data['desiredPosition'] = filtfilt(b, a, self.desiredPosition, axis=0)
        smoothed_data['desiredAcceleration'] = filtfilt(b, a, self.desiredAcceleration, axis=0)
        
        return smoothed_data
        
    def plotVelocity(self)->None:
        """Plot the joints velocity recorded by the sensors."""
        sns.set(style="whitegrid")
        fig, axes = plt.subplots(3, 3, figsize=(10, 6))
        for i in range(7):
            ax = axes[i // 3, i % 3]
            sns.lineplot(ax=ax, x=np.arange(len(self.velocity[:, i])), y= self.velocity[:, i],linewidth=0.5)
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Velocity")
            ax.set_title(f'Joint {i+1}')
        fig.suptitle('Joints Recorded Velocity', fontsize=11)
        plt.show()
        
    def plotPosition(self)->None:
        """Plot the joints position recorded by the sensors."""
        sns.set(style="whitegrid")
        fig, axes = plt.subplots(3, 3, figsize=(10, 6))
        for i in range(7):
            ax = axes[i // 3, i % 3]
            sns.lineplot(ax = ax, x = np.arange(len(self.position[:, i])), y=self.position[:, i],linewidth=0.5)
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Position")
            ax.set_title(f'Joint {i+1}')
        fig.suptitle('Joints Recorded Position', fontsize=10)
        fig.tight_layout(rect = [0, 0, 1, 0.95])
        plt.show()
        
    def plotTorque(self)->None:  
        """Plot the joints torque"""
        sns.set(style="whitegrid") 
        fig, axes = plt.subplots(3, 3, figsize=(10, 6))
        for i in range(7):
            ax = axes[i // 3, i % 3]
            sns.lineplot(ax=ax, x=np.arange(len(self.torque[:, i])), y=self.torque[:, i],linewidth=0.5)
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Torque (N.m)")
            ax.set_title(f'Joint {i+1}')
        fig.suptitle('Joints Recorded Torque', fontsize=11)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()
        
    def plotCurrent(self)-> None:   
        """ """ 
        sns.set(style="whitegrid")
        fig, axes = plt.subplots(3, 3, figsize=(10, 6))
        for i in range(7):
            ax = axes[i // 3, i % 3]
            sns.lineplot(ax=ax, x=np.arange(len(self.current[:, i])), y=self.current[:, i],linewidth=0.5)
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Current (mA)")
            ax.set_title(f'Joint {i+1}')
        fig.suptitle('Joints Recorded Current', fontsize=11)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()    
        
        
   
        
        
        
        
        
#
#robot_data.plotCurrent()
#print(robot_data.duration)