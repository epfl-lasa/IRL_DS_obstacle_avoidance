% Evaluate reg reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    regevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Compute values.
r = zeros(T,1);

if nargout >= 2,
    g = zeros(T,Du);
end;
if nargout >= 3,
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
    for t=1:T,
        d2rdudu(t,:,:) = -eye(Du);
    end;
end;
if nargout >= 5,
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end;
if nargout >= 7,
    gfull = zeros(T,T*Du);
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T,
        Hfull(t,(0:(Du-1))*T + t,(0:(Du-1))*T + t) = -eye(Du);
    end;
end;
