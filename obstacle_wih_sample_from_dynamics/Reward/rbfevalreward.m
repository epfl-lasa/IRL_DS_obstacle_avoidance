% Evaluate Gaussian rbf reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    rbfevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Check if this is a visualization call.
if isempty(x) && size(states,2) < max(reward.idx),
    % This visualization call is in Cartesian coordinates.
    r = 0;
    return;
end;

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Compute distances.
d = bsxfun(@minus,reward.pos,states(:,reward.idx));

% Compute value.
r = reward.r*exp(-0.5*reward.width*sum(d.^2,2));

if nargout >= 2,
    % Compute gradient.
    drdx = zeros(T,Dx);
    drdx(:,reward.idx) = reward.width*bsxfun(@times,d,r);
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
    hmap = zeros(Dx,length(reward.idx));
    hmap(sub2ind(size(hmap),reward.idx,1:length(reward.idx))) = 1;
    for t=1:T,
        D = (reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - reward.width*r(t)*eye(length(reward.idx));
        d2rdxdx(t,:,:) = hmap * D * hmap';
    end;
end;

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
        D = permute(d2rdxdx(t,:,:),[2 3 1]);
        Hfull(t,:,:) = dxdu(:,idxs) * D * dxdu(:,idxs)' + sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end;
end;
