% Evaluate the reward function after computing new states.
function [r,drdu,d2rdudu] = checkhessianfun(reward,mdp,mdp_data,s,u,T)

% Reshape control.
if size(u,2) ~= mdp_data.udims,
    u = reshape(u,size(u,1)/mdp_data.udims,mdp_data.udims);
end;

% Evaluate the control.
[states,A,B,~,dsdu,d2sdudu] = feval(strcat(mdp,'control'),mdp_data,s,u);

% Evaluate the reward.
[r,~,~,~,~,~,drdu,d2rdudu] = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,dsdu,d2sdudu);
r = sum(r(T,:));
drdu = permute(sum(drdu(T,:),1),[2 3 1]);
d2rdudu = permute(sum(d2rdudu(T,:,:),1),[2 3 1]);
