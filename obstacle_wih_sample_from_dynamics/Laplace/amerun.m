% Run the approximate MaxEnt algorithm.
function irl_result = amerun(algorithm_params,mdp,mdp_data,features_pt,...
    features_dyn,example_samples,verbosity)

rng(1);

% Concatenate features.
features = [features_dyn features_pt];

% Add additional regularizing feature to ensure the Hessian has the right
% determinant.
reg_feature = struct('type','reg');
features = [features reg_feature];

% Create random initial weights.
theta = randn(length(features),1)*0.01;
theta(1) = 1.0;
theta(end) = 0.0;

% Create sum reward.
reward = struct('type','sum','theta',theta,'features',{features});

% Compute the gradients and Hessians for each example path.
infos = trajinfos(features,mdp,mdp_data,example_samples);

% Determine initial regularization constant.
theta(end) = auglagfindfeasible(@(wt)amecost(wt,infos),length(features),theta);

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.maxIter = 200;
options.MaxFunEvals = 400;
options.display = 'on';
options.TolX = 1.0e-16;
options.progTol = 1.0e-14;
%options.DerivativeCheck = 'on';
%if verbosity == 0,
    options.display = 'none';
%end;

% Run Augmented Lagrangian optimization.
tic;
theta = auglag(@(wt)amecost(wt,infos),length(features),theta,options,verbosity);
total_time = toc;

% Strip reg feature.
features = features(1:(length(features)-1));
theta = theta(1:length(features));
reward.features = features;
reward.theta = theta';

% Return the reward.
irl_result = struct('reward',reward,'total_time',total_time);
