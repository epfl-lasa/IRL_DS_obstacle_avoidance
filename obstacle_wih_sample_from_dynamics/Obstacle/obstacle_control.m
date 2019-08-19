% Without dynamics case
function [states,A,B,invB,dxdu,d2xdudu] = obstacle_control(mdp_data,x,u)

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);

% To evaluate controls, first compute the Cartesian thrust directions and
% Cartesian state.
thrust = u * mdp_data.motor_basis;
pt0 = [x(:,1) x(:,end)];

% Now evaluate the Cartesian positions.
%states = bsxfun(@plus,cumsum(thrust.^3,1),pt0);
pts = bsxfun(@plus,cumsum(thrust,1),pt0);

% Convert Cartesian positions to states.
states = pts * mdp_data.sensor_basis;

% Now compute the Jacobian.
if nargout >= 2
    % First compute the Jacobian of points to thrust, which is block
    % triangular.
    dpdt = zeros(2*T,2*T);
    for i=1:2
        %dpdt((i-1)*T + (1:T),(i-1)*T + (1:T)) = triu(ones(T)).*repmat(3.0*thrust(:,i)'.^2,T,1);
        dpdt((i-1)*T + (1:T),(i-1)*T + (1:T)) = triu(ones(T));
    end
    
    % Now convert the Jacobian to take us from states to controls.
    dxdu = zeros(Du*T,Dx*T);
    for tu=1:T
        for ts=tu:T
            srcidxs = ts + [0 T];
            srcidxu = tu + [0 T];
            dstidxs = ts + (0:T:(T*(Dx-1)));
            dstidxu = tu + (0:T:(T*(Du-1)));
            dxdu(dstidxu,dstidxs) = mdp_data.motor_basis * dpdt(srcidxu,srcidxs) * mdp_data.sensor_basis;
        end
    end
end

% Now compute the Hessian.
if nargout >= 6
    % First compute the Hessian of points to thrust.
    d2pdtdt = zeros(2*T,2*T,2*T);
    for i=1:2
        for t=1:T
            %mat = diag([6.0*u(1:t,i);zeros(T-t,1)]);
            mat = zeros(T,T);
            d2pdtdt((i-1)*T + (1:T),(i-1)*T + (1:T),(i-1)*T+t) = mat;
        end
    end
    
    % Now convert the Hessian to take us from states to controls.
    % TODO: if for whatever reason this is nonzero, it should be
    % implemented here.
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end

% Create A and B matrices.
if nargout >= 2
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);
    for t=1:T
        dstidxs = t + (0:T:(T*(Dx-1)));
        dstidxu = t + (0:T:(T*(Du-1)));
        A(:,:,t) = eye(Dx);
        B(:,:,t) = dxdu(dstidxu,dstidxs)';
        invB(:,:,t) = pinv(B(:,:,t));
    end
end
