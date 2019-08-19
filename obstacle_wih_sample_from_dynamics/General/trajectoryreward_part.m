% Return the reward obtained along trajectory determined by given inputs,
% as well as the gradient of the reward with respect to those inputs.
function [val_,grad] = trajectoryreward_part(u, s, mdp_data, mdp, reward)

% Reshape.
u = reshape(u,size(u,1)/mdp_data.udims,mdp_data.udims);

% Compute states and state Hessian.
[states,A,B] = feval(strcat(mdp,'control'),mdp_data,s,u);

% Compute reward and its gradient.
% [val,drdu] = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,[]);

% need subsampling here
subsample = 1;
max_x = max(states(:,1));
if subsample
    T = length(states);
    % index = linspace(50, T-50, 50);
    % index = floor(index);
    if mdp_data.num_obs == 2
        logi_index = states(:,1) > 0.5 & states(:,1) < 8.5;
        states = states(logi_index);
    else % one obstacle
%         logi_index = states(:,2) > 4.3;
        logi_index = states(:,1) > 0.35*max_x & states(:,1) < 0.65*max_x;
        
        % add contraint in vertical direction
        logi_index_v = states(:,2) > 0.48;
        logi_index = logi_index & logi_index_v;
        %states = states(index, :);
        states = states(logi_index,:);
    end
    
    num_resample = 15;
    index = linspace(1, length(states), num_resample);
    index = floor(index);
    states = states(index,:);
    u = u(1:num_resample,:);
end

val = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,[]);

% Compute gradient of reward with respect to control.
% grad = -drdu(:);

% compute sum of val according to states x coordinate
% move upwards to reduce some computation
% for i = 1:length(states)
%     if states(i,1)<2 || states(i,1)>8
%         val(i) = 0;
%     end
% end
val_ = -sum(val);

% val_ = 0;
% for i = 1:length(u)
%     if (states(i,1)<6.5) && (states(i,1)>3.5)
%         val_ = val_ - val(i);
%     end
% end

grad = 0;