% Add necessary paths for subdirectories.

% External dependencies.
addpath Utilities
addpath Utilities/minFunc
addpath Utilities/plot2svg

% General functionality.
addpath General
addpath Reward
addpath Auglag
addpath FastHess

% Example domains.
addpath Obstacle

% IRL algorithms.
addpath Laplace
addpath GPIRL
addpath MaxEnt
addpath lib_obstacle_avoidance
addpath new
addpath ROS_
addpath result

addpath('/home/swei/Documents/IRL_with_dynamical_system/matlab_gen/msggen')

%rosinit('128.178.145.170','Nodename','IRL_wsp')