function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    onedimevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% the current states
pts = [states(:,1) states(:,end)];

% the penalize direction
if strcmp(reward.direction,'x')
    direction = [1; 0];
    % reward the corresponding component
    pts = [pts(:,1), zeros(T,1)];
    d = bsxfun(@minus, [10,0], pts);
    r = reward.r*exp(-0.5*reward.width*sum(d.^2,2));
    % r = reward.r * -(pts * direction - 10).^2;
elseif strcmp(reward.direction,'y')
    direction = [0; 1];
    % r = reward.r * (pts* direction + 2.2);
    % r = reward.r * -(pts * direction - 2.2).^2;
    pts = [zeros(T,1), pts(:,2)];
    d = bsxfun(@minus, [0,2.2], pts);
    r = reward.r*exp(-0.5*reward.width*sum(d.^2,2));
end

if nargout >= 2
    % Compute gradient
    % drdx = [ones(T,1), zeros(T,1)];
    % drdx = ones(T,1) * direction';
    if strcmp(reward.direction,'x')
        % drdx = -2 * reward.r * (pts * direction - 10);
        drdx = (reward.width*bsxfun(@times,d,r));
    elseif strcmp(reward.direction,'y')
        % drdx = -2 * reward.r * (pts * direction - 2.2);
        drdx = (reward.width*bsxfun(@times,d,r));
    end
    g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
    % Gradients with respect to controls are always zero.
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end

if nargout >= 5
    if strcmp(reward.direction,'x')
        d2rdxdx = zeros(T,Dx,Dx);
        % d2rdxdx(:,1,1) = -2 * reward.r * ones(T,1);
        for t=1:T
            D = ((reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - ...
                reward.width*r(t)*eye(2));
            d2rdxdx(t,:,:) = D;
        end
    elseif strcmp(reward.direction,'y')
        d2rdxdx = zeros(T,Dx,Dx);
        % d2rdxdx(:,2,2) = -2 * reward.r * ones(T,1);
        for t=1:T
            D = ((reward.width^2)*bsxfun(@times,d(t,:),d(t,:)')*r(t) - ...
                reward.width*r(t)*eye(2));
            d2rdxdx(t,:,:) = D;
        end
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
    
    % Compute Hessian.
    % Construct prototypical constant Hessian.
    Hfull = zeros(T,T*Du,T*Du);
end
