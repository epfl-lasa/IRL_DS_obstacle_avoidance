% Run IRL test with specified algorithm and example.
function test_result = runtest(algorithm,algorithm_params,...
    mdp,mdp_params,test_params, example_human, ss_params)

% test_result - structure that contains results of the test
% algorithm - string specifying the IRL algorithm to use
% algorithm_params - parameters of the specified algorithm
% mdp - string specifying example to test on
% mdp_params - string specifying parameters for example
% test_params - general parameters for the test:
%   training_samples (32) - number of example trajectories to query
%   training_sample_lengths (100) - length of each sample trajectory

% Seed random number generators.
rng(1);

% Set default test parameters.
test_params = setdefaulttestparams(test_params); %sturct

% Construct MDP and features.
[mdp_data, reward, features_pt, features_dyn] = feval(strcat(mdp,'build'),mdp_params);

weight_input = ss_params.weight;

example_samples = cell(1);
if nargin > 5
    states_reverse = cell(length(example_human),1);
    for i = 1: length(example_human)
%         states_reverse{i*2-1} = states_{i}(1:floor(lll/2),:);
%         states_reverse{i*2} = flip(states_{i}(floor(lll/2):end,:));
%         states_reverse{i*2} = states_{i}(floor(lll/2):end,:);

        % only use the half trajectory
%         states_reverse{i} = states_{i}(1:floor(lll/4*2),:);      %3/4  
        
        % Filter the trajectory by horizontal coordinate
        threshold_h = 0.7;
        index1 = example_human{i}(:,1) < max(example_human{i}(:,1))*threshold_h;
        % Or too close to the obstacle 
        threshold_v = 4.8;
        index2 = example_human{i}(:,2) > threshold_v;
        nn = ceil(length(example_human{i})/2);
        index2 = index2 | logical([ones(nn,1); 
                 zeros(length(example_human{i})-nn,1)]);
        index_ = index1 & index2;
        
        states_reverse{i} = example_human{i}(index_, :);
    end
    
    example_human = states_reverse;
     
    % remove the demonstration more than 5 
    len_max = 5;

    if length(example_human) > len_max
        konstant =  length(example_human) - len_max;
    else
        konstant = 0;
    end    
    
    if length(example_human) > len_max
        example_human = example_human(length(example_human)-len_max+1:length(example_human),1);
%         example_human = example_human(length(example_human)-len_max:length(example_human)-1,1);
    end

    % Also embed the weight into example_human ...    
    for i = 1:length(example_human)
        example_samples{i}.s = [0, 4.2];
        tocal_u = [0, 4.2; example_human{i}];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if we use split the demonstrations
%         if mod(i,2) == 1
%             tocal_u = [0, 4.2; example_human{i}];
%         elseif mod(i,2) == 0
%            tocal_u = [10, 4.2; example_human{i}];
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        example_samples{i}.u = diff(tocal_u,1);
        example_samples{i}.initu = zeros(length(example_human{i}),2);
        % ee = example_human{i}(12:61,:);
        example_samples{i}.states = example_human{i};
        example_samples{i}.states_draw = example_human{i};
        example_samples{i}.r = 0;
        example_samples{i}.w = weight_input(i+konstant);
%         example_samples{i}.w = weight_input(i+konstant-1);
        test_samples = [];
    end
else
    % Get example trajectories.
    [example_samples,test_samples] = sampleexamples(mdp_data,mdp,reward,...
        test_params,test_params.verbosity);
end
% % Copy discretization settings.
% algorithm_params.grid_cells_state = test_params.cells_state;
% algorithm_params.grid_cells_action = test_params.cells_action;
% algorithm_params.grid_action_quad = test_params.action_quad;

% Run IRL algorithm.
irl_result = feval(strcat(algorithm,'run'), algorithm_params, mdp, mdp_data,...
    features_pt, features_dyn, example_samples, test_params.verbosity);

% Evaluate IRL result by resynthesizing trajectories.
% irl_result.example_samples = example_samples;
% irl_result.test_samples = test_samples;

% embed some parameters into the struct test_params,
test_params.num_train_demo = ss_params.num_train_demo;
test_params.indicator = ss_params.indicator;

[irl_result.example_samples, irl_result.test_samples, b_reward] = ...
    resampleexamples(mdp_data, mdp, irl_result.reward, reward, test_params,... % mdp is string.
                     example_samples, test_samples, test_params.verbosity);

% Evaluate metrics.
% test_metrics = evaluatemetrics(example_samples,irl_result.example_samples,...
%     test_samples,irl_result.test_samples,reward,irl_result.reward,irl_result);
test_metrics = 0;

%%%%%%%%%%%
% here make the the true reward to be plotted...
% b_reward is what changed inside the resanpleexample function

% disp(irl_result.reward.features{1,2}.gp.inv_widths)

irl_result.reward = b_reward;
%%%%%%%%%%%

% Return result.
test_result = struct('irl_result',irl_result,'reward',reward,... 
    'example_samples',{example_samples},'test_samples',{test_samples},...
    'mdp_data',mdp_data,'mdp_params',mdp_params,'mdp',mdp,...
    'algorithm',algorithm,'test_metrics',test_metrics,...
    'features_pt',{features_pt},'features_dyn',{features_dyn});
