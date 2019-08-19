% Fill in default parameters for the MaxEnt algorithm.
function algorithm_params = maxentdefaultparams(algorithm_params)

% Create default parameters.
default_params = struct(...
    'lsfeatures',1,...
    'seed',0);

% Set parameters.
algorithm_params = filldefaultparams(algorithm_params,default_params);
