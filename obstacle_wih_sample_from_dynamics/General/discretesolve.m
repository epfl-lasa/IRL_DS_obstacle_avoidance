% Solve MDP with discretization.
function discrete_mdp = discretesolve(T,mdp_data,mdp,reward,test_params)

% Compute discrete MDP.
discrete_mdp = discretizemdp(mdp,mdp_data,[],test_params.cells_state,test_params.cells_action,...
                             test_params.action_quad);

% Compute reward function.
R = zeros(discrete_mdp.states,discrete_mdp.actions);
allstates = discrete_mdp.state_vals;
allacts = discrete_mdp.act_vals;
allnstates = zeros(discrete_mdp.states,mdp_data.dims,discrete_mdp.actions);

for a=1:discrete_mdp.actions,
    for s=1:discrete_mdp.states,
        allnstates(s,:,a) = feval(strcat(mdp,'control'),mdp_data,allstates(s,:),allacts(a,:));
    end;
end;

for a=1:discrete_mdp.actions,
    block_size = 128;
    block_count = ceil(discrete_mdp.states/block_size);
    for b=1:block_count,
        sidx = (b-1)*block_size + (1:block_size);
        sidx(sidx > discrete_mdp.states) = [];
        cnt = length(sidx);
        R(sidx,a) = feval(strcat(reward.type,'evalreward'),reward,...
            mdp_data,zeros(1,mdp_data.dims),repmat(allacts(a,:),cnt,1),allnstates(sidx,:,a));
    end;
end;

% Solve for value function.
value_function = zeros(discrete_mdp.states,T);
valrow = max(R,[],2);
value_function(:,T) = valrow;
for t=(T-1):-1:1,
    valrow = max(R + sum(discrete_mdp.sa_p.*valrow(discrete_mdp.sa_s),3),[],2);
    value_function(:,t) = valrow;
end;

% Store reward and value.
discrete_mdp.R = R;
discrete_mdp.V = value_function;
