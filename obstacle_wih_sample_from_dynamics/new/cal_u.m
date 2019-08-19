function [u, initu] = cal_u(s, states)
%CAL_U Summary of this function goes here
%   Detailed explanation goes here
N = size(states,1);
initu = zeros(N,2);

states = [s; states]; 
u = diff(states,1,1);
% u = [rand(1,2); u];
% u = [[0 0]; u];

end

