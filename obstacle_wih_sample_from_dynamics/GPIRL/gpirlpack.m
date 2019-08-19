% Place parameters from specified GP into parameter vector.
function x = gpirlpack(gp,u,wts)

% Add the reward parameters.
x = [wts;u];

% Add ARD kernel parameters.
if gp.learn_ard,
    x = vertcat(x,gp.inv_widths');
end;

% Add noise hyperparameter if we are learning it.
if gp.learn_noise,
    x = vertcat(x,gp.noise_var);
end;

% Add RBF variance hyperparameter if we are learning it.
if gp.learn_rbf,
    x = vertcat(x,gp.rbf_var);
end;
