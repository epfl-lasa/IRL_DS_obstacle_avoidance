% For forward learning with learned dynamics
function [states,A,B,invB,dxdu,d2xdudu] = obstacle_f_control(mdp_data, x, u)
% T = size(u,1);
% x = [0,2.2];
% u = [1.6*ones(T,1), 3*ones(T,1)];

% INPUT: 
%   x 1x2 
%   u 1x2 or nx2
% note, u is on rho and sf space. -> action space
% the x is cartisian space, -> state space 

% ReLU
relu = 0;
if relu
    uu = u;
    du = ones(length(u),2);
    boundsf = 0.9;
    u(uu(:,1)<0.1,1) = 0.1;
    u(uu(:,2)<boundsf,2) = boundsf;
    
    du(uu(:,1)<0.1,1) = 0;
    du(uu(:,1)>=0.1,1) = 1;
    du(uu(:,2)<boundsf,2) = 0;
    du(uu(:,2)>=boundsf,2) = 1;
end

% soft ReLU
softrelu = 0;
if softrelu
    uu = u;
    du = ones(length(u),2);
    u = log(1+exp(uu))+0.9;
    du = 1./(1+exp(-uu)); % 
end

if ~relu && ~softrelu
    du = ones(length(u),2);
end

if isempty(mdp_data.obs_params.opt_sim)
    options = check_options();
else
    options = check_options(mdp_data.obs_params.opt_sim); 
end
obs = options.obstacle;

for n=1:length(obs)
    % x_obs{n} = obs{n}.x0;
    if ~isfield(obs{n},'extra')
        obs{n}.extra.ind = 2;
        obs{n}.extra.C_Amp = 0.01;
        obs{n}.extra.R_Amp = 0.0;
    end
end

b_contour = 0;

XT = [0;0]; % the target point, % dosen't matter at all

% unpack the variable from state
xi = x;
states = x;
x = x';
fn_handle = mdp_data.obs_params.fn_handle;
% xd_obs = [0;0]; % the velocity of obstacle

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);


% if have series of inputs
for k = 1:length(u)
%     disp(k)
    rho = u(k,1);
    sf = u(k,2);

    obs{1}.rho = rho;
    obs{1}.sf = [sf; sf];
    
    if length(obs) >= 2
        obs{2}.rho = rho;
        obs{2}.sf = [sf; sf];
    end

    xd = fn_handle(x-XT);  % x is the location at n-1 time
    %x1 = x(1); x2 = x(2);

    xd_obs = zeros(2, 1);
    
    %a = mdp_data.obs_params.opt_sim.obstacle{1}.a;
    %x0 = mdp_data.obs_params.opt_sim.obstacle{1}.x0;
    % if (((x1-x0(1))/a(1))^2 + ((x2-x0(2))/a(2))^2) >= 1*1.1
        % get new xd from modulation
        [xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is , xd_obs
%     disp(xd)
    %else
    %    xd = [0;0];
    %end

    % get new xd from modulation
    %[xd, b_contour] = obs_modulation_ellipsoid(x, xd, obs, b_contour);% varargin is empty

    % get new x
    x = x + xd*options.dt;

    states(k,:) = x';
end


%%
% Now compute the Jacobian.
if nargout >= 2
    
    % calculate the d_lambda_d_states first
    d_lambda = zeros(Dx,2, T);
    d_E = zeros(Du, Du, Dx);
    d_invE = zeros(Du, Du, Dx);
    d_D = zeros(Du, Du, Dx);

    % d_lambda = [d_lamba_d_x, d_lamda_d_y, d_lambda_d_rho, d_lambda_d_sf]

    % then calculate A and B matrices
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);

    % some constant 
    dyn = [0.3;0]; % related to dynamics
    a = mdp_data.obs_params.opt_sim.obstacle{1}.a;

    states_ = [xi(1,:);states]; % for the for loop
    % states_ = states;
    for t = 1:T
        x1 = states_(t, 1);
        x2 = states_(t, 2);
        rho = u(t,1);
        sf = u(t,2);

        % x0 = mdp_data.obs_params.opt_sim.obstacle{1}.x0;
        % if (((x1-x0(1))/a(1))^2 + ((x2-x0(2))/a(2))^2) <= 1*1.1
        %     continue
        % end  
        
        % subtracte the center of the obstacle
        x1 = x1 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(1);
        x2 = x2 - mdp_data.obs_params.opt_sim.obstacle{1}.x0(2);

        % calculate the gamma
        gamma = (x1/sf/a(1))^2 + (x2/sf/a(2))^2;
        % gamma = (x1/sf)^2 + (x2/sf)^2;
        
        % calculate the lambda1 and lambda2, D
        lambda1 = 1 - (gamma)^(-1/rho);
        lambda2 = -lambda1 + 2;
        D = [lambda1, 0; 0, lambda2];

        % calculate the d_lambda to states and actions
        % d_lambda1/d_x1
        d_lambda(1, 1, t) = 2*x1/(rho*sf^2*a(1)^2)*...
            (gamma)^(-1/rho-1) ;
%         d_lambda(1, 1, t) = 2*x1/(rho*sf^2)*(gamma)^(-1/rho-1) ;
        % d_lambda1/d_x2
        d_lambda(2, 1, t) = 2*x2/(rho*sf^2*a(2)^2)*...
            (gamma)^(-1/rho-1) ;
%         d_lambda(2, 1, t) = 2*x2/(rho*sf^2)*(gamma)^(-1/rho-1) ;
        % d_lambda1/d_rho
        d_lambda(3, 1, t) = -(gamma)^(-1/rho)*log(gamma)*rho^(-2);
        % d_lambda1/d_sf
        d_lambda(4, 1, t) = -2/rho*((x1/a(1))^2 + (x2/a(2))^2)^(-1/rho)...  % it is not gamma inside..
            *(sf)^(2/rho-1);
%         d_lambda(4, 1, t) = -2/rho*(x1^2 + x2^2)^(-1/rho)*(sf)^(2/rho-1);

        % d_lambda2/d_x1
        d_lambda(1, 2, t) = -d_lambda(1, 1, t);
        % d_lambda2/d_x2
        d_lambda(2, 2, t) = -d_lambda(2, 1, t);
        % d_lambda2/d_rho
        d_lambda(3, 2, t) = -d_lambda(3, 1, t);
        % d_lambda2/d_sf
        d_lambda(4, 2, t) = -d_lambda(4, 1, t);

        % some constant
        E = 2/sf^2*[x1/a(1)^2, x2/a(2)^2;...
                    x2/a(2)^2, -x1/a(1)^2]; % with the front constant related to sf..
%         E = 2/sf^2*[x1, x2;...
%                     x2, -x1];

       const_1 = (x1^2/a(1)^4 + x2^2/a(2)^4);
%         const_1 = (x1^2 + x2^2);
        
        invE = 1/const_1*sf^2/2*[x1/a(1)^2, x2/a(2)^2;...  % the inverse, remember inverse the constant multiplied!
                                 x2/a(2)^2, -x1/a(1)^2];
%         invE = 1/const_1*sf^2/2*[x1, x2;...  % the inverse, remember inverse the constant multiplied!
%                                  x2, -x1];

        % compute the derivative in matrix multiplcation form
        % d_E/d_x1
        d_E(:,:,1) = 2/sf^2*[1/a(1)^2, 0; 0 -1/a(1)^2];
%         d_E(:,:,1) = 2/sf^2*[1, 0; 0 -1];
        % d_E/d_x2
        d_E(:,:,2) = 2/sf^2*[0, 1/a(2)^2; 1/a(2)^2, 0];
%         d_E(:,:,2) = 2/sf^2*[0, 1; 1, 0];
        % d_E/d_rho
        d_E(:,:,3) = 0;
        %d_E/d_sf
        d_E(:,:,4) = -2/sf*E; % 2/sf^2* is already included

        % d_invE/d_x1
        d_invE(:,:,1) = -2*x1/a(1)^4*(const_1)^(-2)*[x1/a(1)^2, x2/a(2)^2; ...
            x2/a(2)^2, -x1/a(1)^2]*sf^2/2 + 1/const_1*sf^2/2*[1/a(1)^2, 0; 0 -1/a(1)^2];
%         d_invE(:,:,1) = -2*x1*(const_1)^(-2)*[x1, x2; ...
%             x2, -x1]*sf^2/2 + 1/const_1*sf^2/2*[1, 0; 0 -1];
        % d_invE/d_x2
        % d_invE(:,:,2) = -2*x2/a(2)^4*(const_1)^(-2)*E + 1/const_1*d_E(:,:,2);
        d_invE(:,:,2) = -2*x2/a(2)^4*(const_1)^(-2)*[x1/a(1)^2, x2/a(2)^2; ...
            x2/a(2)^2, -x1/a(1)^2]*sf^2/2 + 1/const_1*sf^2/2*[0, 1/a(2)^2; 1/a(2)^2, 0];
%         d_invE(:,:,2) = -2*x2*(const_1)^(-2)*[x1, x2; x2, -x1]*sf^2/2 + 1/const_1*sf^2/2*[0, 1; 1, 0];
        % d_invE/d_rho
        d_invE(:,:,3) = 0; 
        % d_invE/d_sf
        d_invE(:,:,4) = 1/sf*2*invE;

        % d_D/d_x1
        d_D(:,:,1) = [d_lambda(1,1,t), 0; 
                      0 d_lambda(1,2,t)];
        % d_D/d_x2
        d_D(:,:,2) = [d_lambda(2,1,t), 0; 
                      0 d_lambda(2,2,t)];
        % d_D/d_rho
        d_D(:,:,3) = [d_lambda(3,1,t), 0; 
                      0 d_lambda(3,2,t)];
        % d_D/d_sf
        d_D(:,:,4) = [d_lambda(4,1,t), 0; 
                      0 d_lambda(4,2,t)];

        % together
        % x1 and x2 / x1
        dM_dx1 = d_E(:,:,1)*D*invE*dyn + E*d_D(:,:,1)*invE*dyn + ...
            E*D*d_invE(:,:,1)*dyn;
        dM_dx2 = d_E(:,:,2)*D*invE*dyn + E*d_D(:,:,2)*invE*dyn + ...
            E*D*d_invE(:,:,2)*dyn;
        dM_drho = d_E(:,:,3)*D*invE*dyn + E*d_D(:,:,3)*invE*dyn + ...
            E*D*d_invE(:,:,3)*dyn;
        dM_dsf = d_E(:,:,4)*D*invE*dyn + E*d_D(:,:,4)*invE*dyn + ...
            E*D*d_invE(:,:,4)*dyn;

        % make things clear
        dx1_dx1 = dM_dx1(1)*options.dt + 1;
        dx2_dx1 = dM_dx1(2)*options.dt;
        dx1_dx2 = dM_dx2(1)*options.dt;
        dx2_dx2 = dM_dx2(2)*options.dt + 1;
        % derivative wrt actions
        dx1_drho = dM_drho(1)*options.dt*du(t,1); % du was here for the relu..
        dx2_drho = dM_drho(2)*options.dt*du(t,1);
        dx1_dsf = dM_dsf(1)*options.dt*du(t,2);
        dx2_dsf = dM_dsf(2)*options.dt*du(t,2);

        % pack together
        % A is derivative with respect to previous state
        % B is derivative with respect to action
        A(:,:,t) = [dx1_dx1, dx1_dx2;
                    dx2_dx1, dx2_dx2];

        % B(:,:,t) = [1,0; 0,1; 1,0; 0,1];
        B(:,:,t) = [dx1_drho, dx1_dsf;
                    dx2_drho, dx2_dsf];

        % move 1 more back
%         if i > 1
%             AA(:,:,t) = A(:,:,t-1);
%             BB(:,:,t) = B(:,:,t-1);
%         else
%             AA(:,:,t) = ones(2,2);
%             BB(:,:,t) = ones(2,2);
%         end
        invB(:,:,t) = pinv(B(:,:,t));
    end

    % A = AA;B = BB;
    
    if nargout >= 5
        % Now build the Jacobian out of these matrices.
        dxdu = zeros(Du*T,Dx*T);
        uidx = (0:T:(T*(Du-1)));
        xidx = (0:T:(T*(Dx-1)));
        for c=1:T
            % First, compute the top part of this block row.
            for r=1:(c-1)
                dxdu(r + uidx, c + xidx) = dxdu(r + uidx, (c-1) + xidx)*A(:,:,c)';
            end
            % Now write the diagonal block.
            dxdu(c + uidx, c + xidx) = B(:,:,c)';
        end
    end
end

% Now compute the Hessian.
if nargout >= 6
    
    % First compute the Hessian of points to thrust.
    %{
    d2pdtdt = zeros(2*T,2*T,2*T);
    for i=1:2
        for t=1:T
            %mat = diag([6.0*u(1:t,i);zeros(T-t,1)]);
            mat = zeros(T,T);
            d2pdtdt((i-1)*T + (1:T),(i-1)*T + (1:T),(i-1)*T+t) = mat;
        end
    end
    %}
    
    % Now convert the Hessian to take us from states to controls.
    % TODO: if for whatever reason this is nonzero, it should be
    % implemented here.
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
    
end
