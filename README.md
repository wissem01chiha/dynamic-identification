<div align="center">

## Robotic Dynamics Identification Toolkit
[![Build Status](https://github.com/Justintime50/python-template/workflows/build/badge.svg)](https://github.com/Justintime50/python-template/actions)
[![Coverage Status](https://coveralls.io/repos/github/Justintime50/python-template/badge.svg?branch=main)](https://coveralls.io/github/Justintime50/python-template?branch=main)
[![Licence](https://img.shields.io/github/license/justintime50/python-template)](LICENSE)
</div>

The main objective of this project is to provide an extensive framework for modeling and identificationof the dynamics of 
collaborative serial manipulators.The code extends the classical linear identification methods (IDDIM, CLOE, ...) to non linear dynamics
The available solutions, which can be found here [softwares](/docs/README.md), do not support advanced modeling of friction, stiffness, noise, 
 and robotic joint actuator/transmission systems. This project also aims to provide a comprehensive interface for state-space modeling 
 approaches and transfer functions, rather than standard inverse dynamics equations. Regading the complxity of this models, it offers a method to 
 solve and optimize them,  to find the best trajectory directions and joints configurations that excite the robot dynamics.
### Getting started

### Package structure
all software base structures and functions are in the [source](/src/) folder, specific robot configuration/functions scripts are in [exemple](/exemple/) folder.
### Features
- trajectory data processing
- compute optimized excitation trajectories with non-linear optimization
- compute manipulator state space model 
- state space identification based methods 
- computation of manipulators transfer functions 
- dynamic modeling with effects of frictions, actuator inertia, joint stiffness, and torque offset.
- generation of optimal exciting trajectories.
- calculation of physically consistent standard inertial parameters.
### Documentation
The computation of basic rigid body algorithms for the manipulator is done with [Pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/). For more information, refer to [Pinocchio's GitHub page](https://github.com/stack-of-tasks/pinocchio) or [pinocchio cheat sheet](docs/pinocchio_sheet.pdf).  

see [documentation](docs/README.md) file.
### Examples
all simulation scripts examples are in the [exemple](/exemple/)
 the, at the the for [kinova Gen3-R07](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf) robot can be found in the [examples](exemple/kinova/) directory.
### Prerequisites
the code was tested sucessfuly on the flowing platform : Windows 11 ( python 3.12.4 - miniconda 4.5.0 - cuda 12.50)

- CUDA Compilation Toolkit > 11
- Python interpreter > 3.12
- Conda > 4.5.0
### Installation 
the code will be soon available to installation with python package magners like pip or conda, 
now:
create a new conda environment and install all the dependencies:  
```shell
conda env create -f environment.yml
conda activate dynamapp
```
## Tests
.. on Linux platforms:
```shell 
  chmod u+x run_tests.sh
  ./run_tests.sh
``` 
to run all package tests 
```shell
python3 run_tests.py 
``` 
### References
- **A three-loop physical parameter identification method of robot manipulators considering physical feasibility and nonlinear friction mode**, *Tangzhong Song, Lijin Fang,Guanghui Liu, Hanyu Pang*, 2024
- **Comprehensive modeling and identification of nonlinear joint dynamics for collaborative industrial robot manipulators**, *Emil Madsen, Oluf Skov Rosenlund, David Brandt, Xuping Zhang*, 2020
- **Robotics Modelling Planning and Control**, *Bruno Siciliano, Lorenzo Sciavicco Luigi Villani,Giuseppe Oriolo*, 
- **Global Identification of Joint Drive Gains and Dynamic Parameters of Robots** , *Maxime Gautier*, *Sebastien Brio*, 2014
- **Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots**, *Maxime Gautier, Wisama khalil*, 1990
- **Efficient Dynamic Computer Simulation of Robotic Mechanisms**, *M.W.Walker*, *D. E. Orin*, 1982  
- **Inertial Parameter Identification in Robotics: A Survey**, *Quentin Leboutet, Julien Roux, Alexandre Janot, Julio Rogelio, Gordon
Cheng*, 2021
- **Practical Modeling and Comprehensive System Identification of a BLDC Motor**, *Changle Xiang, Xiaoliang Wang, Yue Ma, and Bin Xu*, 2015
- **Identiable Parameters and Optimum Congurations for Robots Calibration**, *W. Khalil, M. Gautier and Ch. Enguehard*, 2009, *Robotica*
- **Recursive identification of certain structured time varying state-space models**, *M.H. Moazzam T. Hesketh, D.J.Clements*, 1997 
- **Comparison Between the CLOE Method and the DIDIM Method for Robots Identification**, *Alexandre Janot, Maxime Gautier, Anthony Jubien, and Pierre Olivier Vandanjon*, 2014
- **Robot Joint Modeling and Parameter Identification Using the Clamping Method**, *Christian Lehmann ∗ Bjorn Olofsson, Klas Nilsson, Marcel Halbauer, Mathias Haage, Anders Robertsson, Olof Sornmo, Ulrich Berger*, 2013
- **Fundamentals of friction modeling**, *Farid Al-Bender*, 2015
- **Constrained State Estimation - A Review**, *Nesrine Amor, Ghualm Rasool, and Nidhal C. Bouaynaya*, arXiv, 2022
- **The Pinocchio C++ library**,*J.Carpentier, G.Saurel, G.Buondonno, J.Mirabel, F.Lamiraux, O.Stasse, N.Mansard*, 2019

### Development
See the [CONTRIBUTING](CONTRIBUTING.md) guide.
### License
See [LICENSE](LICENSE) file.


