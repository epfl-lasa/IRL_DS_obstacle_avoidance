% Optimize a trajectory starting at a specified point.
function [states,u,r] = reoptimizetrajectory(example,mdp_data,mdp,reward,true_reward,restarts)

if nargin < 6
    restarts = 1;
end

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.maxIter = 1000;
options.MaxFunEvals = 1000;
options.display = 'on';
options.TolX = 1.0e-16;
options.TolFun = 1.0e-8;
options.Display = 'off';
%options.DerivativeCheck = 'on';

if size(restarts,2) > 1
    % Set number of iterations.
    options.maxIter = restarts(2);
    options.MaxFunEvals = restarts(2);
end

% Optimize the states.
if isfield(mdp_data,'optimizes')
    % Run task-specific optimization.
    [u,r] = feval(strcat(mdp,'optimize'),example.s,example.u,mdp_data,mdp,reward,options);
    u = u(:);
else
    % Run minFunc.
    [u,r] = minFunc(@(p)trajectoryreward(p,example.s,mdp_data,mdp,reward),example.u(:),options);
end

if restarts(1) > 1
    if size(restarts,2) > 2
        % Set number of iterations.
        options.maxIter = restarts(3);
        options.MaxFunEvals = restarts(3);
    end
    % Run restarts.
    for i=2:restarts(1)
        newu = feval(strcat(mdp,'samplecontrols'),size(example.u,1),mdp_data);
        if isfield(mdp_data,'optimizes')
            % Run task-specific optimization.
            [newu,newr] = feval(strcat(mdp,'optimize'),example.s,newu,mdp_data,mdp,reward,options);
            newu = newu(:);
        else
            [newu,newr] = minFunc(@(p)trajectoryreward(p,example.s,mdp_data,mdp,reward),newu(:),options);
        end
        if newr < r
            r = newr;
            u = newu;
        end
    end
end
r = -trajectoryreward(u,example.s,mdp_data,mdp,true_reward);
u = reshape(u,size(example.u,1),mdp_data.udims);
states = feval(strcat(mdp,'control'),mdp_data,example.s,u);
