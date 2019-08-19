% Evaluate Gaussian rbf reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    idevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

if isempty(x) && size(states,2) < max(reward.idx),
    % If there is no initial state, this is just a visualization call.
    r = 0;
    g = [];
    drdu = [];
    d2rdudu = [];
    drdx = [];
    d2rdxdx = [];
    gfull = [];
    Hfull = [];
    return;
end;

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Compute reward.
r = reward.r*states(:,reward.idx);

if nargout >= 2,
    % Compute gradient.
    drdx = zeros(T,Dx);
    drdx(:,reward.idx) = reward.r;
    g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end;

if nargout >= 3,
    % Gradients with respect to controls are always zero.
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end;

if nargout >= 6,
    % drdx has already been computed.
    d2rdxdx = zeros(T,Dx,Dx);
end;

% If necessary, compute and return the full Hessian.
if nargout >= 7,
    % Compute gfull.
    % Convert gradient to T x TD matrix.
    drdxmat = zeros(T,T*Dx);
    for i=1:Dx,
        drdxmat(:,(i-1)*T + (1:T)) = diag(drdx(:,i));
    end;

    % Compute gradient with respect to controls.
    gfull = drdxmat * dxdu';
    
    % Compute Hfull.
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T,
        idxs = (0:(Dx-1))*T + t;
        Hfull(t,:,:) = sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end;
end;
