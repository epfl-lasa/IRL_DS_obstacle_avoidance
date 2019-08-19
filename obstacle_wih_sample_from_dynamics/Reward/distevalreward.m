% Evaluate interstate distance reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    distevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

if isempty(x),
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

% This reward penalizes the square of the control, thus preventing
% excessively long leaps.
r = reward.r * sum(u(:,reward.idx).^2,2);

if nargout >= 2,
    % Compute gradient with respect to controls.
    g = zeros(T,Du);
    g(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
end;

if nargout >= 3,
    drdu = zeros(T,Du);
    drdu(:,reward.idx) = 2.0 * reward.r * u(:,reward.idx);
    d2rdudu = zeros(T,Du,Du);
    mask = zeros(Du,1);
    mask(reward.idx,1) = 1;
    for t=1:T,
        d2rdudu(t,:,:) = reward.r * 2.0 * diag(mask);
    end;
end;

if nargout >= 5,
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end;

if nargout >= 7,
    % Compute gradient with respect to controls.
    gfull = zeros(T,T*Du);
    for i=reward.idx,
        gfull(:,(i-1)*T + (1:T)) = diag(2.0*u(:,i)) * reward.r;
    end;
    % Compute Hessian.
    % Construct prototypical constant Hessian.
    hessmat = zeros(T,T,T);
    for i=1:T,
        hessmat(i,i,i) = 2;
    end;
    hessmat = reward.r * hessmat;
    Hfull = zeros(T,T*Du,T*Du);
    for d=reward.idx,
        strt = (d-1)*T;
        Hfull(1:T,(strt+1):(strt+T),(strt+1):(strt+T)) = ...
            hessmat;
    end;
end;
