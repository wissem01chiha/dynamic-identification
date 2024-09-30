## pyDynaMapp : Python Dynamics Mapping Package For Robotics


[![Version](https://anaconda.org/chiha/pydynamapp/badges/version.svg)](https://anaconda.org/chiha/pydynamapp)
[![Version](https://anaconda.org/chiha/pydynamapp/badges/latest_release_date.svg)](https://anaconda.org/chiha/pydynamapp)
[![Version](https://anaconda.org/chiha/pydynamapp/badges/platforms.svg)](https://anaconda.org/chiha/pydynamapp)
[![Version](https://anaconda.org/chiha/pydynamapp/badges/license.svg)](https://anaconda.org/chiha/pydynamapp)
[![Version](https://anaconda.org/chiha/pydynamapp/badges/downloads.svg)](https://anaconda.org/chiha/pydynamapp)


The primary goal of this project is to offer a flexible framework for modeling and identifying serial manipulators, including the incorporation of nonlinear effects such as friction, stiffness, and backlash in robot joints. Additionally, a predictive feedback control mechanism is being developed to compensate these effects, making the framework suitable for collaborative robotics applications.

### Getting started
### Prerequisites
The code was tested successfully on the following platform:

- **Windows 11**:
  - python 3.12.4
  - miniconda 4.5.0
  - cuda 12.50 (optional)

> **⚠️ Warning:** The package has not been tested or built for Unix-like systems (macOS, Linux). Please refer to the [Unix installation](#installation/Unix) section for more details.

### Installation 
#### Windows
> **Note:** If Anaconda is installed on your system, skip the first step. Conda is the only Python package manager supported at the moment.


1. Download and install [Miniconda](https://docs.anaconda.com/miniconda/miniconda-install/). For Windows installation, refer to the [Miniconda Windows Installation Guide](https://docs.anaconda.com/miniconda/miniconda-install/).

2. Install the package using the following command:
    ```shell
    pip install -i https://pypi.anaconda.org/chiha/simple pydynamapp
    ```

#### Using Git:

1. Clone the repository:  
    ```shell
    git clone https://github.com/wissem01chiha/dynamic-identification
    ```
2. Install the following enviroment dependencies manually:  
    - `numpy==1.26.4`
    - `pinocchio==2.7.1`
    - `scipy==1.14.0`
    - `nlopt==2.7.1`
    - `pyyaml==6.0.2`
    - `pandas==2.2.2`
    - `seaborn==0.13.2`
    - `matplotlib==3.8.4`   

    -- using conda:
    ```shell
    conda config --add channels conda-forge
    conda install pinocchio=2.7.1
    conda install nlopt=2.7.1
    ...
    ```
    -- or for Windows platforms, run the command to install all requirements:
    ```shell
     conda env create -f environment.yml 
    ```
    by defualt it create a virtual enviroment named **dynamapp** and place the python packages in it.
   
> **⚠️ Warning:** The Pinocchio release 2.7.1 is the officially supported version. Newer releases (3.x and above) may cause errors. If you encounter any issues, please open an [issue](https://github.com/wissem01chiha/dynamic-identification/issues).
    

3. Run all tests to check the installation:

    ```shell
    python3 run_tests.py
    ```

for any bugs encountered during installation, please make sure to open an issue at: [Issues](https://github.com/wissem01chiha/dynamic-identification/issues)


#### Unix
 We strongly recommend using Conda to create a virtual environment with: 
 
```shell
conda env create -f environment.yml
conda activate dynamapp
```
an then follow the same instruction in [using git](#installation/usinggit) to insatll code dependancies in the virtual enviroment.
run test to check installation: 
.. on Linux platforms:
```shell 
  chmod u+x run_tests.sh
  ./run_tests.sh
``` 
for any bugs encountered during installation, please make sure to open an issue at: [Issues](https://github.com/wissem01chiha/dynamic-identification/issues)

### Package Structure

- The **`script`** folder contains executable identification scripts. Settings for each algorithm are passed through command-line arguments.
- The **`dynamics`** folder contains all direct and inverse dynamics model computations, state-space representation, and regressor formulation.
- The **`viscoelastic`** folder contains all base classes for contact models.
- The package relies on the [URDF](https://wiki.ros.org/urdf) file format in the **`robot`** folder to represent the rigid body tree of the robot and contains basic inertial and geometric parameters. For more information, refer to:
  - [Pinocchio Documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/)
  - [Pinocchio's GitHub Page](https://github.com/stack-of-tasks/pinocchio)
  - [Pinocchio Cheat Sheet](docs/pinocchio_sheet.pdf)

- **create a robot model:** Use a URDF file along with a configuration file. The configuration file should follow the same layout as the [Kinova config file](https://github.com/wissem01chiha/dynamic-identification/blob/main/pyDynaMapp/robot/kinova/config.yml).

    ```python
    from pyDynaMapp.dynamics import Robot

    kinova = Robot(urdf_file_path, configuration_file_path)
    
    # Example: Compute mass and Coriolis matrices for home position
    C = kinova.computeCoriolisMatrix()
    M = kinova.computeMassMatrix()
    
    # Example: Compute the joint torques vector for home position
    T = kinova.computeGeneralizedTorques()
    ```

- **create a state-space representation :** 

    ```python
    from pyDynaMapp.dynamics import StateSpace
    import numpy as np

    kinova_ss = StateSpace(urdf_file_path, configuration_file_path)
    
    # Example: Compute the state-space matrices at a given configuration using a state vector x
    x = np.random.rand(14)
    A, B, C, D = kinova_ss.model.computeStateMatrices(x)
    
    # Example: Compute the state-space matrices at a given configuration using position and velocity vectors
    q = np.random.rand(7)
    qp = np.random.rand(7)
    A, B, C, D = kinova_ss.model.computeStateMatrices(q, qp)

### Documentation
official documentation is not yet released and is a work in progress.   
for more information, please visit the [pyDynaMapp-Anaconda](https://anaconda.org/chiha/pydynamapp) page.

### References
- **[A three-loop physical parameter identification method of robot manipulators considering physical feasibility and nonlinear friction mode](https://link.springer.com/article/10.1007/s11071-024-09755-w)**, *Tangzhong Song, Lijin Fang,Guanghui Liu, Hanyu Pang*, 2024
- **[Comprehensive modeling and identification of nonlinear joint dynamics for collaborative industrial robot manipulators](https://www.sciencedirect.com/science/article/pii/S0967066120300988)**, *Emil Madsen, Oluf Skov Rosenlund, David Brandt, Xuping Zhang*, 2020
- **[Robotics Modelling Planning and Control](https://link.springer.com/book/10.1007/978-1-84628-642-1)**, *Bruno Siciliano, Lorenzo Sciavicco Luigi Villani,Giuseppe Oriolo*, 
- **[Global Identification of Joint Drive Gains and Dynamic Parameters of Robots](https://link.springer.com/article/10.1007/s11044-013-9403-6)** , *Maxime Gautier*, *Sebastien Brio*, 2014
- **[Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots](https://ieeexplore.ieee.org/document/56655)**, *Maxime Gautier, Wisama khalil*, 1990
- **[Efficient Dynamic Computer Simulation of Robotic Mechanisms](https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/104/3/205/428542/Efficient-Dynamic-Computer-Simulation-of-Robotic?redirectedFrom=fulltext)**, *M.W.Walker*, *D. E. Orin*, 1982  
- **[Inertial Parameter Identification in Robotics: A Survey](https://www.mdpi.com/2076-3417/11/9/4303)**, *Quentin Leboutet, Julien Roux, Alexandre Janot, Julio Rogelio, Gordon
Cheng*, 2021
- **[Practical Modeling and Comprehensive System Identification of a BLDC Motor](https://onlinelibrary.wiley.com/doi/10.1155/2015/879581)**, *Changle Xiang, Xiaoliang Wang, Yue Ma, and Bin Xu*, 2015
- **[Identiable Parameters and Optimum Congurations for Robots Calibration](https://www.researchgate.net/publication/232021354_Identifiable_Parameters_and_Optimum_Configurations_for_Robots_Calibration)**, *W. Khalil, M. Gautier and Ch. Enguehard*, 2009, *Robotica*
- **[Recursive identification of certain structured time varying state-space models](https://digital-library.theiet.org/content/journals/10.1049/ip-cta_19971383)**, *M.H. Moazzam T. Hesketh, D.J.Clements*, 1997 
- **[Comparison Between the CLOE Method and the DIDIM Method for Robots Identification](https://ieeexplore.ieee.org/document/6728671)**, *Alexandre Janot, Maxime Gautier, Anthony Jubien, and Pierre Olivier Vandanjon*, 2014
- **[Robot Joint Modeling and Parameter Identification Using the Clamping Method](https://www.sciencedirect.com/science/article/pii/S1474667016343889)**, *Christian Lehmann ∗ Bjorn Olofsson, Klas Nilsson, Marcel Halbauer, Mathias Haage, Anders Robertsson, Olof Sornmo, Ulrich Berger*, 2013
- **[Fundamentals of friction modeling](https://www.researchgate.net/publication/266016929_Fundamentals_of_friction_modeling)**, *Farid Al-Bender*, 2015
- **[Constrained State Estimation - A Review](https://arxiv.org/pdf/1807.03463v3)**, *Nesrine Amor, Ghualm Rasool, and Nidhal C. Bouaynaya*, arXiv, 2022
- **[The Pinocchio C++ library](https://ieeexplore.ieee.org/document/8700380)**,*J.Carpentier, G.Saurel, G.Buondonno, J.Mirabel, F.Lamiraux, O.Stasse, N.Mansard*, 2019

### Development
See the [CONTRIBUTING](CONTRIBUTING.md) guide.
### License
See [LICENSE](LICENSE) file.
