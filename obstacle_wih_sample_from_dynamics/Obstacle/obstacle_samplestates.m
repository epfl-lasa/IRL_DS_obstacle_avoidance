% Sample some random states in the Objectworld.
% sample the starting states
function s = obstacle_samplestates(n,mdp_data)

% s = bsxfun(@times,rand(n,2),mdp_data.bounds) * mdp_data.sensor_basis;
 
% s = [0, 4.2];% + 4.0*rand(1)];

s = [0, 4.2];
