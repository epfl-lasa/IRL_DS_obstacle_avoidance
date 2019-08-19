% Reward formed from OptV value function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    optvevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Check if this is a visualization call.
if isempty(x) && size(states,2) == 2 && size(states,2) ~= mdp_data.dims,
    % This visualization call is already in Cartesian coordinates.
    % For now, just return zero and don't crash.
    r = 0.0*states(:,1);
    if nargout > 1,
        error('Visualization call only supports a single return argument');
    end;
    return;
end;

T = size(states,1);
Du = size(u,2);
Dx = size(states,2);

% This is a little hack to give OptV the step cost.
if Dx == 2,
    step_cost = 40.0;
else
    step_cost = 10.0;
end;

if nargout > 2,
    error('OptV reward does not support returning anything beyond r and g');
end;

% Precompute w3.
w3 = permute(reward.w,[2 3 1]);

% Convert theta.
F = reward.F;
TPF = 1 + Dx + 0.5*Dx*(Dx+1);
theta3 = permute(reshape(reward.theta,TPF,F),[3 4 2 1]);

% Set x.
%x = [x; states(1:(T-1),:)];
x = states;
%{
% Precompute f(xpr) for xpr.
% This is N x 1 x F matrix
xf = exp(sum(bsxfun(@times,optvs(permute(x,[1 3 2])),theta3),4));
xf = min(xf,1.0e20);
% This is N x 1 x F x D matrix
xfg = permute(bsxfun(@times,xf,sum(bsxfun(@times,optvs(permute(x,[1 3 2]),1),theta3),4)),[1 2 3 5 4]);
% Now apply normalization.
xfn = max(sum(xf,3),1.0e-20);
xfg = bsxfun(@minus,bsxfun(@rdivide,xfg,xfn),bsxfun(@times,bsxfun(@rdivide,xf,xfn.^2),sum(xfg,3)));
xf = bsxfun(@rdivide,xf,xfn);
r = -sum(bsxfun(@times,xf,w3),3);
return;
%}

% Compute successor states.
allxpr = zeros(size(x,1),size(reward.act_vals,1),size(x,2));
pallxpr = zeros(size(x,1),size(reward.act_vals,1));
for i=1:size(x,1),
    % Evaluate each action.
    for a=1:size(reward.act_vals,1),
        pt = feval(strcat(mdp_data.mdp,'control'),mdp_data,x(i,:),reward.act_vals(a,:));
        allxpr(i,a,:) = pt(1,:);
        pallxpr(i,a) = exp(-step_cost*sum(reward.act_vals(a,:).^2,2));
    end;
end;

% Normalize probabilities.
pallxpr = bsxfun(@rdivide,pallxpr,sum(pallxpr,2));

% Precompute f(xpr) for xpr.
% This is N x 1 x F matrix
xf = exp(sum(bsxfun(@times,optvs(permute(x,[1 3 2])),theta3),4));
xf = min(xf,1.0e20);
% This is N x 1 x F x D matrix
xfg = permute(bsxfun(@times,xf,sum(bsxfun(@times,optvs(permute(x,[1 3 2]),1),theta3),4)),[1 2 3 5 4]);
% Now apply normalization.
xfn = max(sum(xf,3),1.0e-20);
xfg = bsxfun(@minus,bsxfun(@rdivide,xfg,xfn),bsxfun(@times,bsxfun(@rdivide,xf,xfn.^2),sum(xfg,3)));
xf = bsxfun(@rdivide,xf,xfn);
    
% Precompute f(allxpr) for allxpr.
% This is N x A x F matrix
allxprf = exp(sum(bsxfun(@times,optvs(allxpr),theta3),4));
allxprf = min(allxprf,1.0e20);
% This is N x A x F x D matrix
allxprfg = permute(bsxfun(@times,allxprf,sum(bsxfun(@times,optvs(allxpr,1),theta3),4)),[1 2 3 5 4]);
% Now apply normalization.
allxprfn = max(sum(allxprf,3),1.0e-20);
allxprfg = bsxfun(@minus,bsxfun(@rdivide,allxprfg,allxprfn),bsxfun(@times,bsxfun(@rdivide,allxprf,allxprfn.^2),sum(allxprfg,3)));
allxprf = bsxfun(@rdivide,allxprf,allxprfn);

% Compute numerator.
num = -sum(bsxfun(@times,xf,w3),3);

% Compute denominator.
asums = -sum(bsxfun(@times,allxprf,w3),3);
maxx = max(asums,[],2);
asums = bsxfun(@minus,asums,maxx);
Gp = bsxfun(@times,pallxpr,exp(asums));
G = sum(Gp,2);

% Compute r.
%r = - num + log(G) + maxx - step_cost*sum(u.^2,2);
r = num - log(G) - maxx - step_cost*sum(u.^2,2);

% Compute gradient if necessary.
if nargout >= 2,
    % The first part: the gradient of the numerator.
    gradn = permute(-sum(bsxfun(@times,xfg,w3),3),[1 4 2 3]); % T x Dx array.
    gradg = permute(bsxfun(@rdivide,sum(bsxfun(@times,Gp,sum(bsxfun(@times,allxprfg,w3),3)),2),G),[1 4 2 3]); % T x Dx array.
    %drdx = -(gradn+gradg);
    drdx = (gradn+gradg);
    g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
    g = g - 2.0*step_cost*u;
end;
