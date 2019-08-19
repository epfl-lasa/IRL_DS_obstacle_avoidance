function pre_process_data(path)

if nargin < 1
%     path = '/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/eight_subject/Jun_12_01/testb2/';
    path = '/home/swei/Documents/result/temp/July26/3-1/';
end

for j = 1:58
    filename = ['data_', num2str(j), '.mat'];
    path_ = strcat(path, filename);
    s = load(path_);
    states = s.states;
    T = length(states);

    % resacle the states
    rangex = [min(states(:,1)) max(states(:,1))];c
    rangey = [min(states(:,2)) max(states(:,2))];
    a = [abs(rangex(1)), 0];
    
    % reverse x
    states_r = (states+repmat(a,length(states),1))./(rangex(2)-rangex(1)).*10;
    
    if (states_r(T,1)<states_r(1,1))
        states_r(:,1) = -states_r(:,1) + 10;
        % states_r = flipud(states_r);
    end
    
    % put the first point at 0,4.2
    dd = 4.2 - states_r(1,2);
    states_r(:,2) = states_r(:,2) + dd;
          
    % sub sampleing
    lll = 50; % set the length to be 50
    index = linspace(lll, T-lll, 50);
    index = floor(index);
    states_tbl = states_r(index, :);
    
    % split the trajectory to 2 parts .. 
%     lll = length(states_tbl);
%     threshold_h = 0.7;
%     states_tbl = states_tbl(states_tbl(:,1) < max(states_tbl(:,1))*threshold_h, :);
        
    % use the trajectory for learning
    
    states_{j} = states_tbl;
%    [rho, sf] = obstacle_test(2,1,1,1,'sim', states_);

end

weight_input = [0.67, 0.51, 0.83, 0.59, 0.00, 0.58, 0.30, 0.00, 0.00, ...
0.85];
% save('/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/eight_subject/Jun_05_02/testb2/weight_input.mat', 'weight_input')
% weight_input = [0.00, 0.64, 0.17, 0.55, 0.74, 0.37, 0.56, 0.60, 0.00, 0.03];
% weight_input = [0.49, 0.31, 0.51, 0.14, 0.0, 0.43, 0.33, 0.31, 0.58, 0.0];
% weight_input = [0.61, 0.68, 0.0, 0.0, 0.0, 0.15, 0.68, 0.85, 0.71, 0.21];

% weight_input = [0.0, 0.22, 0.0, 0.0, 0.0, 0.05, 0.0, 0.72, 0.76, 0.38];
% weight_input = [0.3, 0.03, 0.57, 0, 0, 0.91, 0, 0, 0, 0.71];
% weight_input = [0, 0, 0.31, 0, 0.04, 0.11, 0, 0.29, 0, 0.3];
% weight_input = [0, 0, 0.17, 0.25, 0.11, 0, 0.23, 0, 0, 0];

weight_input = 1 - weight_input;

% == after the first two subject, read weight from mat.
weight_input = load([path, 'weight_input.mat']);
weight_input = weight_input.weight_input;

SIGMOID = 1;
if SIGMOID
    % sigmoid
    weight_input2 = weight_input*10;
%         weight_input = sigmoid(weight_input,5,2);
    weight_input2 = sigmoid(weight_input2,5,1);
end

node(states_, weight_input2, path)

end
