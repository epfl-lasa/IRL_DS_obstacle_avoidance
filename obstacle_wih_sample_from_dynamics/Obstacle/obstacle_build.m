% Construct the Objectworld MDP structures.
function [mdp_data,reward,features_pt,features_dyn] = obstacle_build(mdp_params)

% mdp_params - parameters of the objectworld
% mdp_data - standard MDP definition structure with object-world details
% reward - reward object for this objectworld
% features - cell array of features

% Fill in default parameters.
mdp_params = obstacle_defaultparams(mdp_params);

% Set random seed.
rng(mdp_params.seed);

% Compute motor and sensor bases.
motor_basis = zeros(mdp_params.motors,2);
sensor_basis = zeros(2,mdp_params.sensors);
for i=1:mdp_params.motors
    % Choose angle.
    angle = (pi * 0.5) * (i-1) / (mdp_params.motors - 1);
    % Write basis vector.
    motor_basis(i,:) = [cos(angle) sin(angle)];
end

for i=1:mdp_params.sensors
    % Choose angle.
    angle = (pi * 0.5) * (i-1) / (mdp_params.sensors - 1);
    % Write basis vector.
    sensor_basis(:,i) = [cos(angle); sin(angle)];
end

% Compute pseudoinverse of sensor basis.
sensor_pseudoinverse = sensor_basis'*inv(sensor_basis*sensor_basis');

% Compute state and action bounds.
sp = linspace(0,pi*0.5,mdp_params.sensors);
sbounds = [zeros(1,mdp_params.sensors); (cos(sp) + sin(sp))*mdp_params.size];
abounds = [-ones(1,mdp_params.motors)*0.6; ones(1,mdp_params.motors)*0.6];

% Place the objects.
objects = struct('pos',[],'c1',[],'c2',[]);
if mdp_params.fixed_pattern == 1
    % Place objects in flower petals fixed pattern.
elseif mdp_params.fixed_pattern == 2
    % Place objects in hill and valley fixed pattern.
elseif mdp_params.fixed_pattern == 3
    % Place the obstacle
    objects(1) = struct('pos',[0.5, 0.4]*mdp_params.size,'c1',-5,'c2',1);
    objects(2) = struct('pos',[0.5, 0.4]*mdp_params.size,'c1',15,'c2',1);
    objects(3) = struct('pos',[0.5, 0.4]*mdp_params.size,'c1',-25,'c2',1);
    
%     objects(2) = struct('pos',[0.35, 0.0]*mdp_params.size,'c1',2,'c2',1);
%     objects(3) = struct('pos',[0.65, 0.0]*mdp_params.size,'c1',2,'c2',1);
    
    objects(length(objects)+1) = struct('pos',0,'c1',1,'c2',1);
    objects(length(objects)+1) = struct('pos',[1.0, 0.4]*mdp_params.size,'c1',5,'c2',1);
elseif mdp_params.fixed_pattern == 4
    % two obstacles
%     objects(1) = struct('pos',[0.4, 0.4]*mdp_params.size,'c1',-5,'c2',1);
%     objects(2) = struct('pos',[0.6, 0.6]*mdp_params.size,'c1',15,'c2',1);
    objects(1) = struct('pos',mdp_params.centers(1,:)*mdp_params.size,'c1',-5,'c2',1);
    objects(2) = struct('pos',mdp_params.centers(2,:)*mdp_params.size,'c1',-5,'c2',1);
else
    % Place objects at random.
    for i=1:mdp_params.objects
        pos = rand(1,2)*mdp_params.size;
        % Select c1 based on probabilities in the parameters.
        samp = rand(1,1);
        if samp < mdp_params.c1_1prob
            c1 = 1;
        elseif samp < mdp_params.c1_1prob+mdp_params.c1_2prob,
            c1 = 2;
        else
            c1 = 2+randi(mdp_params.c1-2,1);
        end
        c2 = randi(mdp_params.c2,1);
        objects(i) = struct('pos',pos * sensor_basis,'c1',c1,'c2',c2);
    end
end

% Create MDP data structure.
mdp_data = struct(...
    'mdp','obstacle_',...
    'dims',size(sensor_basis,2),...
    'udims',size(motor_basis,1),...
    'sbounds',sbounds,'abounds',abounds,...
    'sensor_pseudoinverse',sensor_pseudoinverse,...
    'sensor_basis',sensor_basis,...
    'motor_basis',motor_basis,...
    'bounds',mdp_params.size*ones(1,2),...
    'objects',objects,...
    'obs_params', mdp_params.obs_params,...
    'start_point', mdp_params.obs_params.x0);

if mdp_params.fixed_pattern == 4
    mdp_data.num_obs = 2;
else
    mdp_data.num_obs = 1;
end

% Construct feature map.
[reward,features_pt,features_dyn] = obstacle_features(mdp_params,mdp_data);
