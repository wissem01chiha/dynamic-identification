%% Data extraction (first traj)
data_gen3_traj1 = readtable("trajectory_recorded\traj0.csv");
smoothing_factor1 = [0.18 0.18 0.15 0.15 0.11 0.11 0.15];

%% Data extraction (second traj)
data_gen3_traj2 = readtable("trajectory_recorded\RecordTrajectory_quadratic.csv");
smoothing_factor2 = [0.4 0.04 0.09 0.23 0.1 0.2 0.76];

%% Data extraction and filtering (third traj)
data_gen3_traj3 = readtable("trajectory_recorded\traj1.csv");
smoothing_factor3 = [0.21 0.1 0.27 0.2 0.06 0.15 0.19];
