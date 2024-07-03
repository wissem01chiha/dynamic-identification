import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import logging
import math
import json
import yaml

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def wrap2deg(angle):
    """Wrap angle to the interval [-180, 180] degrees."""
    wrapped_angle = angle - 360 * np.floor((angle + 180) / 360)
    return wrapped_angle

def wrap2deg(angle):
    """Wrap angle to the interval [-180, 180]."""
    return angle - 360 * np.floor((angle + 180) / 360)

def rad2deg(angle_radians):
    """Converts an angle given in radians to degrees."""
    return angle_radians * (180 / math.pi)

def wrapXpi(angle, X):
    """Wrap angle to the interval [-X*pi, X*pi]."""
    return angle - X * math.pi * np.floor((angle + X / 2 * math.pi) / (X * math.pi))

def deg2rad(angle_degrees):
    """Converts an angle given in degrees to radians."""
    radians = angle_degrees * (math.pi / 180)
    return radians

def checkSkewSymmetric(matrix):
    """Check if the input matrix is skew-symmetric."""
    matrix_transpose = np.transpose(matrix)
    status = np.array_equal(matrix, -matrix_transpose)
    return status

def struct2json(structData, filename):
    """Saves a Python dictionary to a JSON file."""
    if not isinstance(structData, dict):
        logger.error('The first input argument must be a Python dictionary')
    if not isinstance(filename, str) or filename == '':
        logger.error('The second input argument must ba non-empty string representing the filename')
    try:
        with open(filename, 'w') as file:
            json.dump(structData, file, indent=4)
    except IOError:
        logger.error(f'Could not create or open the file "{filename}" for writing')

def columnVector(vec):
    """Convert a vector to a column vector."""
    if not np.ndim(vec) == 1:
        logger.error("Input must be a vector.")
    colVector = vec[:, np.newaxis]
    return colVector

def matrix2Text(matrix, filename):
    """Write the values of a matrix to a text file."""
    try:
        with open(filename, 'w') as file:
            rows, cols = matrix.shape
            for i in range(rows):
                for j in range(cols):
                    file.write(f'  {matrix[i, j]}  ')
                file.write('\n')
    except IOError:
        logger.error(f'Cannot open or write to file: {filename}')
    
def yaml2dict(yamlFilePath) -> dict:
    """
    Get parameters from the config YAML file and return them as a 
    dictionary.
    
    Args:
        yamlFilePath (str): Path to the YAML file.
    """
    try:
        with open(yamlFilePath, 'r') as file:
            dic = yaml.safe_load(file)
        return dic
    except FileNotFoundError:
        logger.error(f"Error: File '{yamlFilePath}' not found.")
        return {}
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML file '{yamlFilePath}': {e}")
        return {}

def plotArray(array: np.ndarray,title=None,ylabel = None) -> None:
    """
    Given an ( n * m )  data array where n >> m, plot each coloum data 
    in sperate subplots .

    Args:
        - array: numpy ndarray
    """
    N = array.shape[0]
    if array.ndim ==1 :
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111)
        sns.lineplot(ax=ax, x=np.arange(N), y=array, linewidth=0.5, color='blue')
        ax.set_xlabel("Time (ms)", fontsize=9)
        if not(ylabel is None):
            plt.ylabel(ylabel, fontsize=9)
    elif array.ndim == 2:
        ndof = min(array.shape[1],array.shape[0])
        if not(ndof == array.shape[1]):
            array = np.transpose(array)
        fig, axes = plt.subplots(3, 3, figsize=(10, 6), dpi=100)
        axes = axes.flatten()
        for i in range(ndof):
            ax = axes[i]
            sns.lineplot(ax=ax, x=np.arange(N), y=array[:, i], linewidth=0.5,color='blue')
            ax.set_xlabel("Time (ms)", fontsize=9)
            if not(ylabel is None):
                ax.set_ylabel(ylabel, fontsize=9)
            ax.set_title(f'Joint {i+1}', fontsize=9)

        for j in range(ndof, len(axes)):
            fig.delaxes(axes[j])
            
    if title != None: 
        fig.suptitle(title, fontsize=9)   
    plt.tight_layout()
        
         
def plot2Arrays(array1: np.ndarray, array2: np.ndarray, legend1=None, legend2=None,title=None,
               color1='red', color2='blue') -> None:
    """
    Given two (n * m) data arrays where n >> m, plot each column data
    from both arrays in separate subplots.

    Args:
        - array1: First numpy ndarray
        - array2: Second numpy ndarray
        - legend1: Legend for the first array  
        - legend2: Legend for the second array  
        - color1: Line color for the first array  
        - color2: Line color for the second array  
        - title: Title for the figure  
    """
    assert array1.shape == array2.shape, "Arrays should have the same size."
    ndof = min(array1.shape[1],array1.shape[0])
    N = array1.shape[0]
    fig, axes = plt.subplots(3, 3, figsize=(10, 6), dpi=100)
    axes = axes.flatten()
    
    for i in range(ndof):
        ax = axes[i]
        sns.lineplot(ax=ax, x=np.arange(N), y=array1[:, i], linewidth=0.5, color=color1, label=legend1)
        sns.lineplot(ax=ax, x=np.arange(N), y=array2[:, i], linewidth=0.5, color=color2, label=legend2)
        ax.set_xlabel("Time (ms)", fontsize = 9)
        ax.set_title(f'Joint {i+1}', fontsize = 9)
        if legend1 or legend2:
            ax.legend(fontsize = 6)
    
    for j in range(ndof, len(axes)):
        fig.delaxes(axes[j])
    if title:
        fig.suptitle(title, fontsize=9)
    plt.tight_layout()
    
def plot3Arrays(array1: np.ndarray, array2: np.ndarray, array3: np.ndarray, 
                    legend1=None, legend2=None, legend3=None, 
                    title=None, color1='red', color2='blue', color3='green') -> None:
    """
    Given three (n * m) data arrays where n >> m, plot each column data
    from all arrays in separate subplots.

    Args:
        - array1: First numpy ndarray
        - array2: Second numpy ndarray
        - array3: Third numpy ndarray
        - legend1: Legend for the first array  
        - legend2: Legend for the second array  
        - legend3: Legend for the third array  
        - color1: Line color for the first array  
        - color2: Line color for the second array  
        - color3: Line color for the third array  
        - title: Title for the figure  
    """
    ndof = array1.shape[1]
    N = array1.shape[0]
    fig, axes = plt.subplots(3, 3, figsize=(12, 8), dpi=100)
    axes = axes.flatten()
    
    for i in range(ndof):
        ax = axes[i]
        sns.lineplot(ax=ax, x=np.arange(N), y=array1[:, i], linewidth=0.5, color=color1, label=legend1)
        sns.lineplot(ax=ax, x=np.arange(N), y=array2[:, i], linewidth=0.5, color=color2, label=legend2)
        sns.lineplot(ax=ax, x=np.arange(N), y=array3[:, i], linewidth=0.5, color=color3, label=legend3)
        ax.set_xlabel("Time (ms)", fontsize=9)
        ax.set_title(f'Joint {i+1}', fontsize=9)
        if legend1 or legend2 or legend3:
            ax.legend(fontsize=6)
    
    for j in range(ndof, len(axes)):
        fig.delaxes(axes[j])
    if title:
        fig.suptitle(title, fontsize=9)
    plt.tight_layout()
