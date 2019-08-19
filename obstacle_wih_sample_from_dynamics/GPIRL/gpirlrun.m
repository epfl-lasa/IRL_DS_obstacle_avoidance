% Run the approximate GPIRL algorithm.
function irl_result = gpirlrun(algorithm_params, mdp, mdp_data, features_pt,...
    features_dyn, example_samples, verbosity)

% Get algorithm parameters.
algorithm_params = gpirldefaultparams(algorithm_params);

% Set seeds.
rng(algorithm_params.seed);

% Check if we should initialize with AME.
if algorithm_params.ame_init
    ame_result = amerun(algorithm_params.ame_params,mdp,mdp_data,features_pt,...
        features_dyn,example_samples,verbosity);
end

% Add additional regularizing feature to ensure the Hessian has the right
% determinant.
reg_feature = struct('type','reg');
features_dyn = [features_dyn reg_feature];

% Create initial reward from features with random initial weights.
wts = ones(length(features_dyn),1);
wts(end) = 0.0; % Set this to zero for now, will decide on weight later.

% Create priors for linear features.
dyn_priors = repmat({'none'},1,length(features_dyn));
dyn_prior_wts = zeros(1,length(features_dyn));

% Compute the gradients and Hessians for each example path.
infos_dyn = trajinfos(features_dyn,mdp,mdp_data,example_samples);
infos_pt = trajinfos(features_pt,mdp,mdp_data,example_samples);

% First set up the supporting states.
F = length(features_pt);
if strcmp(algorithm_params.sample_mode,'data')
    F_u = [];
    curState = 0;
    N = length(infos_pt);
    for i=1:N
        % Set state indices.
        stateIndices = (1:size(infos_pt{i}.f,1))';

        % Store features for this trajectory.
        F_u = [F_u; infos_pt{i}.f(stateIndices,:)];

        % Add to state count.
        curState = curState + size(stateIndices,1);
    end
    
    % Now subsample if necessary.
    if size(F_u,1) > algorithm_params.samples
        idx = randperm(size(F_u,1));
        F_u = F_u(idx(1:algorithm_params.samples),:);
    end
elseif strcmp(algorithm_params.sample_mode,'random')
    totalStates = algorithm_params.samples;
    pts = feval(strcat(mdp,'samplestates'),totalStates,mdp_data);
    F_u = zeros(totalStates,F);
    for f=1:F
        F_u(:,f) = feval(strcat(features_pt{f}.type,'evalreward'),features_pt{f},mdp_data,[],zeros(totalStates,mdp_data.udims),pts,[],[],[],[]);
    end
end

% Set up initial hyperparameters and create GP structure.
gp = struct('F_u',F_u);
gp.inv_widths = gpirlhpxform(algorithm_params.ard_init*ones(1,F),[],algorithm_params.ard_xform,3);
gp.rbf_var = gpirlhpxform(algorithm_params.rbf_init,[],algorithm_params.ard_xform,3);
gp.noise_var = gpirlhpxform(algorithm_params.noise_init,[],algorithm_params.noise_xform,3);
gp.ard_prior_wt = algorithm_params.ard_prior_wt;
gp.noise_prior_wt = algorithm_params.noise_prior_wt;
gp.rbf_prior_wt = algorithm_params.rbf_prior_wt;
gp.gamma_shape = algorithm_params.gamma_shape;
gp.ard_prior = algorithm_params.ard_prior;
gp.noise_prior = algorithm_params.noise_prior;
gp.rbf_prior = algorithm_params.rbf_prior;
gp.ard_xform = algorithm_params.ard_xform;
gp.noise_xform = algorithm_params.noise_xform;
gp.rbf_xform = algorithm_params.rbf_xform;
gp.learn_ard = algorithm_params.learn_ard;
gp.learn_noise = algorithm_params.learn_noise;
gp.learn_rbf = algorithm_params.learn_rbf;
gp.dyn_priors = dyn_priors;
gp.dyn_prior_wts = dyn_prior_wts;

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.maxIter = 200;
options.MaxFunEvals = 400;
options.display = 'on';
options.TolX = 1.0e-16;
options.progTol = 1.0e-14;
%options.DerivativeCheck = 'on';
if verbosity == 0
    options.display = 'none';
end
options.display = 'none';

% Uncomment the line below to check the Hessian.
%gpirlhesstest(example_samples,features_pt,features_dyn,mdp_data,gp,u,wts,2);

% Initialize u.
if algorithm_params.ame_init
    % Set u using weights returned by AME.
    % Take the last F entries of theta.
    ame_weights = ame_result.reward.theta((length(features_dyn)):end);
    % Compute rewards.
    u = sum(bsxfun(@times,F_u,ame_weights),2);
    % Set weights on dynamic terms.
    wts(1:(length(features_dyn)-1)) = (ame_result.reward.theta(1:(length(features_dyn)-1)))';
else
    % Set u at random.
    u = rand(size(F_u,1),1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%
% add discounting factor in the infos_pt.
% infos_pt{i}
base_discounting = 1;
for i = 1:length(infos_pt)
    infos_pt{i}.discount = base_discounting^(length(infos_pt)-i);
end

% weight_vector = [1, 0.7, 0.5, 0.1];
% weight_vector = [1, 0.05, 0.01, 0.01];

% weight_vector = [1, 0.9, 0.1, 0.1];
% for i = 1:length(weight_vector)
%     infos_pt{i}.discount = weight_vector(i);
% end

% use the weight input from EEG side
for i = 1:length(example_samples)
    infos_pt{i}.discount = example_samples{i}.w;
end
%%%%%%%%%%%%%%%%%%%%%%%%%

% Pick weight on regularization term to ensure the objective is defined.
wts(end) = auglagfindfeasible(@(wt)gpirlcost(wt,gp,infos_dyn,infos_pt),length(wts),gpirlpack(gp,u,wts));

% Run Augmented Lagrangian optimization.
tic;
bestll = Inf;
for r=1:algorithm_params.restarts % which is 1
    if r > 1
        % Pick new u and new weight.
        u = rand(size(F_u,1),1);
        wts(end) = auglagfindfeasible(@(wt)gpirlcost(wt,gp,infos_dyn,infos_pt),length(wts),gpirlpack(gp,u,wts));
    end
    % Pack the parameters.
    params = gpirlpack(gp,u,wts);
    % Run the optimization. --- THE optimization process
    params = auglag(@(wt)gpirlcost(wt,gp,infos_dyn,infos_pt),length(wts),params,options,verbosity);
    % Evaluate score.
    tparams = params;
    tparams(length(wts)) = 0.0; % Remove regularizing feature before weighting.
    ll = gpirlcost(tparams,gp,infos_dyn,infos_pt);
    if ll < bestll
        bestll = ll;
        bestparams = params;
    end
    % Print outcome.
    fprintf(1,'Continuous GPIRL iteration %i: %f\n',r,-ll);
end
% Unpack the parameters.
[gp,u,wts] = gpirlunpack(gp,bestparams);
total_time = toc;

% Store auxiliary GP stuff.
[gp.K_uu,gp.invK_uu,gp.alpha] = gpirlgpprior(gp,u);

% Strip reg feature.
features_dyn = features_dyn(1:(length(features_dyn)-1));
wts = wts(1:length(features_dyn));

% Construct the reward.
% The reward is the sum of a GP term and a weighted sum of dynamic
% features.
% Construct gp reward.
gp_feature = struct('type','gp','u',u,'gp',gp,'features',{features_pt});
% Construct the final sum reward.
reward = struct('type','sum','theta',[wts' 1],'features',{[features_dyn gp_feature]});

% Construct IRL result.
irl_result = struct('reward',reward,'total_time',total_time);
if algorithm_params.ame_init
    irl_result.ame_result = ame_result;
end
