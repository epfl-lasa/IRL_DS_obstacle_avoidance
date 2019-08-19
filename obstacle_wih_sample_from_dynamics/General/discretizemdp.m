% Discretize MDP state space.
function discrete_mdp = discretizemdp(mdp,mdp_data,example_samples,cells_state,cells_action,action_quad)

% The returned structure must fill in the following fields:
% sa_p
% sa_s
% state_vals
% act_vals

% Compute state and action bounds.
sbounds = mdp_data.sbounds;
abounds = mdp_data.abounds;
if ~isempty(example_samples),
    % If we have examples, get the bounds from them.
    for i=1:length(example_samples),
        u = example_samples{i}.u;
        s0 = example_samples{i}.s;
        pts = feval(strcat(mdp,'control'),mdp_data,s0,u);
        sbounds(1,:) = min([pts;sbounds(1,:)],[],1);
        sbounds(2,:) = max([pts;sbounds(2,:)],[],1);
        abounds(1,:) = min([u;abounds(1,:)],[],1);
        abounds(2,:) = max([u;abounds(2,:)],[],1);
    end;
end;

% Compute state and action grid cells.
state_vals = buildgrid(sbounds,cells_state,0);
act_vals = buildgrid(abounds,cells_action,action_quad);

% Compute transition probabilities.
states = size(state_vals,1);
actions = size(act_vals,1);
transitions = 2^mdp_data.dims;
sa_p = zeros(states,actions,transitions);
sa_s = zeros(states,actions,transitions);

% Create discrete mdp.
discrete_mdp = struct('sa_s',sa_s,'sa_p',sa_p,...
    'sbounds',sbounds,'abounds',abounds,...
    'cells_state',cells_state,'cells_action',cells_action,...
    'state_vals',state_vals,'act_vals',act_vals,...
    'states',size(state_vals,1),'actions',size(act_vals,1));

% Determine the successor states and interpolate them onto the grid.
for a=1:actions,
    fprintf(1,'Building successor states for action %i of %i\n',a,actions);
    for s=1:states,
        next_state = feval(strcat(mdp,'control'),mdp_data,state_vals(s,:),act_vals(a,:));
        [sa_s(s,a,:), sa_p(s,a,:)] = interpolatestate(next_state,discrete_mdp,mdp_data);
    end;
end;

% Store tables.
discrete_mdp.sa_s = sa_s;
discrete_mdp.sa_p = sa_p;
