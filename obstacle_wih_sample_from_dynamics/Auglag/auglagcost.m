% Compute objective for Augmented Lagrangian computation.
function [val,grad] = auglagcost(params,fun,min_idx,AUGLAG_POWER,USE_EXP,mu,lambda)

if USE_EXP,
    % Convert first entry from log space.
    params(min_idx) = min(1.0e20,exp(params(min_idx)));
end;

% Get objective and gradient from objective.
if nargout == 2,
    [val,grad] = fun(params);
else
    val = fun(params);
end;

if isinf(mu),
    % We've switched off the constrained variable.
    grad(min_idx) = 0.0;
else
    % Add constraint terms.
    val = val + 0.5*mu*(params(min_idx)^(2*AUGLAG_POWER)) - lambda*(params(min_idx)^AUGLAG_POWER);
    if nargout == 2,
        grad(min_idx) = grad(min_idx) + AUGLAG_POWER*mu*(params(min_idx)^(2.0*AUGLAG_POWER - 1)) - AUGLAG_POWER*lambda*(params(min_idx)^(AUGLAG_POWER-1));
    end;
end;

if USE_EXP && nargout == 2,
    % Convert gradient back into log space.
    grad(min_idx) = params(min_idx)*grad(min_idx);
end;
