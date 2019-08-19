% For forward learning with learned dynamics
function [states,A,B,invB,dxdu,d2xdudu] = obstacle_ftwo_control(mdp_data, x, u)

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
    rho2 = u(k,3);
    sf2 = u(k,4);
    
    obs{1}.rho = rho;
    obs{1}.sf = [sf; sf];
    
    if length(obs) >= 2
        obs{2}.rho = rho2;
        obs{2}.sf = [sf2; sf2];
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
    % then calculate A and B matrices
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);
end

% Now compute the Hessian.
if nargout >= 6
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
    
end
