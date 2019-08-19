% Sample example tranjectories from the state space of a given MDP.
function [example_samples,test_samples] = sampleexamples(mdp_data,mdp,reward,test_params,verbose)

% Allocate training samples.
N = test_params.training_samples;
Nt = test_params.test_samples;
T = test_params.training_sample_lengths;
example_samples = cell(1,N);
test_samples = cell(1,Nt);

% Sample example trajectories. -- from dynamics
para = cell(1,1);
T1 = floor(T/16);
T2 = floor(T/8);
T3 = floor(T/4);
T4 = floor(T/2);
% para{1} = [ones(T1,1)*0.5, ones(T1,1)*0.9;...
%             ones(T1,1)*3, ones(T1,1)*1.2;...
%             ones(T2,1)*3, ones(T2,1)*1.5;...
%             ones(T3,1)*8, ones(T3,1)*1.6;...
%             ones(T4,1)*0.5, ones(T4,1)*0.9];
para{1} = [ones(T,1)*2.2, ones(T,1)*1.4];
para{2} = [ones(T,1)*2, ones(T,1)*1.6];
para{3} = [ones(T,1)*4, ones(T,1)*1.5];
para{4} = [ones(T,1)*3, ones(T,1)*1.5];
para{5} = [ones(T,1)*2, ones(T,1)*1.6];
para{6} = [ones(T,1)*4, ones(T,1)*1.5];
para{7} = [ones(T,1)*3, ones(T,1)*1.5];
para{8} = [ones(T,1)*2, ones(T,1)*1.6];

for i=1:N
    if verbose > 0
        fprintf(1,'Preparing example %i of %i\n',i,N);
    end

    % Sample initial state.
    s = feval(strcat(mdp,'samplestates'),1,mdp_data);

    % rho and sf [rho, sf] Nx2
    
    param = para{i};
    
    states = obstacle_control_nod(mdp_data, s, param);
    
    [u, initu] = cal_u(s, states);
    
    r = 0;% i think the r is useless in following learning section
    
    % Create example struct.
    example_samples{i} = struct('s',s,'u',u,'initu',initu,'states',states,'r',r);
end
