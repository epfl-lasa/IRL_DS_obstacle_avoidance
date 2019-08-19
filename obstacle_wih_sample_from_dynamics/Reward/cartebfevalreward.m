% Gaussian rbf in Cartesian space for Objectworld type tasks.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    cartebfevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Convert states to Cartesian space.
pts = [states(:,1) states(:,end)];
    
% Compute distances.
d = bsxfun(@minus,reward.pos,pts);

% Compute value.
% r = reward.r(1)*exp(-0.5*reward.width*sum(d.^2,2)); 
% diag(d*d') is same as sum(d.^2,2)
% E = eye(2);
% E = [0.5,0;0,1];
E = reward.E;
r = reward.r(1)*exp(-0.5*reward.width*diag(d*E*d'));

if nargout >= 2
    % Compute gradient.
    drdx = (reward.width*bsxfun(@times,d*E,r));
    g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
    % Gradients with respect to controls are always zero.
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end

if nargout >= 6
    % drdx has already been computed.
    d2rdxdx = zeros(T,Dx,Dx);
    for t=1:T
        D = ((reward.width^2)*bsxfun(@times,d(t,:)*E,E'*d(t,:)')*r(t) - reward.width*E*r(t)*eye(2));
        d2rdxdx(t,:,:) = D;
    end
end

if nargout >= 7
    % Compute gfull.
    % Convert gradient to T x TD matrix.
    drdxmat = zeros(T,T*Dx);
    for i=1:Dx
        drdxmat(:,(i-1)*T + (1:T)) = diag(drdx(:,i));
    end

    % Compute gradient with respect to controls.
    gfull = drdxmat * dxdu';
    
    % Compute Hfull.
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T
        idxs = (0:(Dx-1))*T + t;
        D = mdp_data.sensor_pseudoinverse*...
            ((reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - reward.width*r(t)*eye(2))*...
            mdp_data.sensor_pseudoinverse';
        Hfull(t,:,:) = dxdu(:,idxs) * D * dxdu(:,idxs)' + sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end
end
