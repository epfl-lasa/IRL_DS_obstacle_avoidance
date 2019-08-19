% Ziebart's Maximum Entropy IRL.
function irl_result = maxentrun(algorithm_params,mdp,mdp_data,features_pt,...
    features_dyn,example_samples,verbosity)

rng(1);

% Fill in default parameters.
algorithm_params = maxentdefaultparams(algorithm_params);

% Concatenate features.
features_all = [features_dyn features_pt];

% Discretize MDP.
discrete_mdp = discretizemdp(mdp,mdp_data,example_samples,...
    algorithm_params.grid_cells_state,algorithm_params.grid_cells_action,...
    algorithm_params.grid_action_quad);

% Initialize variables.
[states,actions,transitions] = size(discrete_mdp.sa_p);

% Build feature membership matrix.
% Note that we must convert backward features over (s_t,a_t) into forward
% features over (s_{t-1},a_t).
features = length(features_all);
F = zeros(states,actions,features);
allstates = discrete_mdp.state_vals;
allacts = discrete_mdp.act_vals;

% This computes features directly.
allnstates = zeros(states,mdp_data.dims,actions);
for a=1:actions,
    for s=1:states,
        allnstates(s,:,a) = feval(strcat(mdp,'control'),mdp_data,allstates(s,:),allacts(a,:));
    end;
end;
for f=1:features,
    for a=1:actions,
        block_size = 128;
        block_count = ceil(states/block_size);
        for b=1:block_count,
            sidx = (b-1)*block_size + (1:block_size);
            sidx(sidx > states) = [];
            cnt = length(sidx);
            F(sidx,a,f) = feval(strcat(features_all{f}.type,'evalreward'),features_all{f},...
                mdp_data,zeros(1,mdp_data.dims),repmat(allacts(a,:),cnt,1),allnstates(sidx,:,a));
        end;
    end;
end;

%{
% This uses sa_p to compute features.
for f=1:features,
    for a=1:actions,
        for s=1:states,
            ss = allstates(s,:);
            sn = allstates(discrete_mdp.sa_s(s,a,:),:);
            an = bsxfun(@minus,sn,ss);
            r = feval(strcat(features_all{f}.type,'evalreward'),features_all{f},...
                mdp_data,zeros(1,mdp_data.dims),an,sn);
            F(s,a,f) = sum(r.*permute(discrete_mdp.sa_p(s,a,:),[3 1 2]));
        end;
    end;
end;
%}

% Discretize the example data and compute feature expectations.
N = length(example_samples);
T = size(example_samples{1}.u,1);
muE = zeros(features,T);
mu_sa = zeros(states,actions,T);

%{
% This computes examples using nearest state and action.
ex_s = zeros(N,T);
ex_a = zeros(N,T);
for i=1:N,
    % Compute states.
    u = example_samples{i}.u;
    s0 = example_samples{i}.s;
    pts = feval(strcat(mdp,'control'),mdp_data,s0,u);
    % Determine closest initial state.
    [~,ex_s(i,1)] = min(sum(bsxfun(@minus,permute(s0,[1 3 2]),permute(discrete_mdp.state_vals,[3 1 2])).^2,3),[],2);
    for t=1:T,
        % Determine the closest state.
        if t < T,
            [~,ex_s(i,t+1)] = min(sum(bsxfun(@minus,permute(pts(t,:),[1 3 2]),permute(discrete_mdp.state_vals,[3 1 2])).^2,3),[],2);
        end;
        % Determine the closest action.
        [~,ex_a(i,t)] = min(sum(bsxfun(@minus,permute(u(t,:),[1 3 2]),permute(discrete_mdp.act_vals,[3 1 2])).^2,3),[],2);
        % Add to expectations.
        muE(:,t) = muE(:,t) + permute(F(ex_s(i,t),ex_a(i,t),:),[3 1 2]);
        mu_sa(ex_s(i,t),ex_a(i,t),t) = mu_sa(ex_s(i,t),ex_a(i,t),t) + 1;
    end;
end;
%}
% This computes examples with multilinear interpolation.
for i=1:N,
    % Compute states.
    u = example_samples{i}.u;
    s0 = example_samples{i}.s;
    pts = feval(strcat(mdp,'control'),mdp_data,s0,u);
    r = zeros(T,features);
    for f=1:features,
        r(:,f) = feval(strcat(features_all{f}.type,'evalreward'),...
                    features_all{f},mdp_data,s0,u,pts);
    end;
    % Interpolate initial state.
    [cur_ss,cur_sp] = interpolatestate(s0,discrete_mdp,mdp_data);
    for t=1:T,
        % Interpolate action.
        [cur_as,cur_ap] = interpolatestate(u(t,:),discrete_mdp,mdp_data,1+algorithm_params.grid_action_quad);
        
        if ~algorithm_params.lsfeatures,
            % Add to expectations.
            muE(:,t) = muE(:,t) + permute(sum(sum(...
                bsxfun(@times,F(cur_ss(:),cur_as(:),:),bsxfun(@times,cur_sp(:),(cur_ap(:))')),...
                    1),2),[3 1 2]);
            mu_sa(cur_ss(:),cur_as(:),t) = mu_sa(cur_ss(:),cur_as(:),t) + bsxfun(@times,cur_sp(:),(cur_ap(:))');
        else
            % Solve least-squares problem among local states.
            FM = reshape(F(cur_ss(:),cur_as(:),:),[size(cur_ss,3)*size(cur_as,3) features])';
            freqs = lsqlin(FM,r(t,:)',[],[],ones(1,size(FM,2)),1,zeros(size(FM,2),1),[],[],optimset('LargeScale','off','Display','off'));
            freqs = reshape(freqs,size(cur_ss,3),size(cur_as,3));
            muE(:,t) = muE(:,t) + permute(sum(sum(...
                bsxfun(@times,F(cur_ss(:),cur_as(:),:),freqs),...
                    1),2),[3 1 2]);
            mu_sa(cur_ss(:),cur_as(:),t) = mu_sa(cur_ss(:),cur_as(:),t) + freqs;
        end;
        
        % Interpolate next state.
        if t < T,
            [cur_ss,cur_sp] = interpolatestate(pts(t,:),discrete_mdp,mdp_data);
        end;
    end;
end;

% Generate initial state distribution for fixed horizon.
initD = zeros(states,1,T);
for t=1:T,
    % Add visitations for this time step.
    initD(:,1,t) = initD(:,1,t) + sum(mu_sa(:,:,t),2);
    if t < T,
        % Subtract visitions for the next time step.
        initD(:,1,t+1) = initD(:,1,t+1) - sum(sparse(discrete_mdp.sa_s(:),1:numel(discrete_mdp.sa_p),...
            reshape(bsxfun(@times,discrete_mdp.sa_p,mu_sa(:,:,t)),numel(discrete_mdp.sa_p),1),...
            size(discrete_mdp.sa_s,1),numel(discrete_mdp.sa_s)),2);
    end;
end;

fun = @(r)maxentcost(r,F,muE,mu_sa,discrete_mdp,initD,T);

% Set up optimization options.
options = struct();
options.Display = 'iter';
options.LS_init = 2;
options.LS = 2;
options.Method = 'lbfgs';
%options.DerivativeCheck = 'on';
if verbosity == 0,
    options.display = 'none';
end;

% Initialize reward.
r = rand(features,1);

% Run unconstrainted non-linear optimization.
tic;
[r,~] = minFunc(fun,r,options);
total_time = toc;

% Create sum reward.
reward = struct('type','sum','theta',r','features',{features_all});

% Return the reward.
irl_result = struct('reward',reward,'total_time',total_time);
