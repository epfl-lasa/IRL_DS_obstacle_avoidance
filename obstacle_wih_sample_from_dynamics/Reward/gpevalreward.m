% Evaluate Gaussian process reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    gpevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

F = length(reward.features);
T = size(states,1);
Du = size(u,2);
Dx = size(states,2);

if nargout > 6
    error('GP reward does not support returning full gradients and Hessians');
end

% Create structure.
info = struct(...
    'linear_dynamics',1,...
    'f',zeros(T,F),...
    'g',zeros(T,F,Du),...
    'gh',zeros(T,F,Dx),...
    'gt',zeros(T,F,Du));

% Build Jacobian.
if nargout >= 2
    info.A = A;
    info.B = B;
end

% Remove field.
if isfield(mdp_data,'defergrad')
    mdp_data = rmfield(mdp_data,'defergrad');
end
for f=1:F
    % Compute the feature.
    if nargout >= 2
        [r,g,drdu,~,drdx,~] = ...
            feval(strcat(reward.features{f}.type,'evalreward'),...
                reward.features{f},mdp_data,x,u,states,A,B);
    else
        r = feval(strcat(reward.features{f}.type,'evalreward'),...
                reward.features{f},mdp_data,x,u,states);
    end

    % Save value.
    info.f(:,f) = r;

%     %%%
%     if f == 4
%         info.f(:,f) = r;
%     else
%         info.f(:,f) = zeros(T,1);
%     end
    %%%
    % Save gradients.
    if nargout >= 2
        info.g(:,f,:) = permute(g,[1 3 2]);
        info.gh(:,f,:) = permute(drdx,[1 3 2]);
        info.gt(:,f,:) = permute(drdu,[1 3 2]);
    end
end

% Kernalize the states.
if nargout >= 3
    % Assume we don't actually want the Hessians.
    d2rdudu = zeros(T,Du,Du);
    d2rdxdx = zeros(T,Dx,Dx);
    [K,g,drdx,drdu] = gpirlgpgrads(zeros(T,1,Du),reward.gp.invK_uu,reward.gp.alpha,info,reward.gp);
elseif nargout >= 2
    [K,g] = gpirlgpgrads(zeros(T,1,Du),reward.gp.invK_uu,reward.gp.alpha,info,reward.gp);
else
    K = gpirlgpgrads(zeros(T,1,Du), reward.gp.invK_uu, reward.gp.alpha, info, reward.gp);
end

% Compute value.
r = K*reward.gp.alpha;

if nargout >= 2
    g = permute(g,[1 3 2]);
end

if nargout >= 3
    drdx = permute(drdx,[1 3 2]);
    drdu = permute(drdu,[1 3 2]);
end
