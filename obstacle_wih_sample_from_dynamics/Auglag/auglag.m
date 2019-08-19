% Run the Augmented Lagrangian algorithm with a constraint to minimize a
% single input variable.
function params = auglag(fun,min_idx,params,options,verbose)

% Constants.
TAU = 0.5; % How much the feasibility must improve to avoid increasing mu.
AUGLAG_POWER = 1; % Exponent to raise constraint to.
REG_TOL = 1.0e-5; % Tolerance for constraint satisfaction.
INIT_MIN = 1.0e-2; % Minimum initial mu.
INIT_MAX = 1.0e2; % Maximum initial mu.
MAX_ITERATIONS = 20;
USE_EXP = 1; % Use exponential transform to enforce positivity.

% Set up optimization options.

% Initialize Augmented Lagrangian mu constant.
% This is based on the NLOPT Augmented Lagrangian initialization.
[funcVal,grad] = fun(params);
%mu = max(INIT_MIN,min(INIT_MAX,2.0*abs(funcVal)/(params(min_idx)^(2*AUGLAG_POWER))));
% the upper line existed in the cioc collection
mu = max(INIT_MIN,min(INIT_MAX,0.1*mean(abs(grad))/abs(AUGLAG_POWER*(params(min_idx)^(2*AUGLAG_POWER-1)))));
lambda = 0.0;
prevreg = params(min_idx);
if verbose > 0
    fprintf(1,'Initializing regularization set to %f, mu = %f\n',params(min_idx),mu);
end

% Suppress warning:
warning off MATLAB:nearlySingularMatrix;

% Run the initial optimization.
if USE_EXP
    params(min_idx) = log(params(min_idx));
end
params = minFunc(@(p)auglagcost(p,fun,min_idx,AUGLAG_POWER,USE_EXP,mu,lambda),params,options);
if USE_EXP
    params(min_idx) = exp(params(min_idx));
end

% Apply Augmented Lagrangian updates until regularization feature goes away.
converged = 0;
iterations = 0;
total_itr = 0;
while (abs(params(min_idx)) > REG_TOL || (converged == 0 && iterations < 2)),
    zparams = params;
    zparams(min_idx) = 0;
    if ~USE_EXP && params(min_idx) <= 0 && ~isinf(fun(zparams))
        % If the parameter is negative, clamp it to zero and switch off
        % lambda, but *only* if this doesn't cause the objective to become
        % infinite.
        params(min_idx) = 0;
        mu = Inf;
        lambda = 0;
    else
        % Advance lambda using the Augmented Lagrangian update rule.
        lambda = lambda - mu*(params(min_idx)^AUGLAG_POWER);
        % Increase mu if constraint violation is not decreasing.
        if abs(params(min_idx)^AUGLAG_POWER) > TAU*abs(prevreg^AUGLAG_POWER) && abs(params(min_idx)) > REG_TOL
            mu = mu*10.0;
        end
    end
    % Store old constraint violation.
    prevreg = params(min_idx);
    % Print status.
    if verbose > 0
        fprintf(1,'Reg feature has weight: %f, setting mu = %f, lambda = %f\n',params(min_idx),mu,lambda);
    end
    % Run optimization and get new results.
    if USE_EXP
        params(min_idx) = log(params(min_idx));
    end
    [params,~,flag] = minFunc(@(p)auglagcost(p,fun,min_idx,AUGLAG_POWER,USE_EXP,mu,lambda),params,options);
    if USE_EXP
        params(min_idx) = exp(params(min_idx));
    end
    % Check convergence.
    converged = flag > 0;
    % Count the number of iterations since we satisfied the constraint.
    if abs(params(min_idx)) > REG_TOL
        iterations = 0;
    else
        iterations = iterations + 1;
    end
    total_itr = total_itr + 1;
    if total_itr >= MAX_ITERATIONS
        if verbose > 0
            fprintf(1,'Maximum number of augmented Lagrangian iterations reached.\n');
        end
        break;
    end
end

% Enable warning:
warning on MATLAB:nearlySingularMatrix;

% Print final weight on reg feature.
if verbose > 0
    fprintf(1,'Reg feature has final weight: %f\n',params(min_idx));
end
