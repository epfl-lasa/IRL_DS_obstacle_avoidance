% Determine regularizing weight necessary to make all Hessians negative definite.
function newwt = auglagfindfeasible(fun,min_idx,params)

% Search to determine what we need to do to make all Hessians negative
% definite.
oldwt = 1.0e-4;
newwt = 2.0e-4;
lastbad = 0.0;
lastgood = -1.0;
while 1
    % Compute the objective value.
    params(min_idx) = newwt;
    val = fun(params);
    
    % When the Hessian is not negative definite, the value is -Inf.
    all_good = val < 1e50;
    
    % Test for termination.
    if all_good && (newwt < 4.0e-4 || abs((newwt-oldwt)/(newwt+oldwt)) < 1.0e-4),
        break
    end
    
    % Store weight.
    oldwt = newwt;
    
    % Change weight.
    if all_good
        % Decrease weight.
        newwt = 0.5*(newwt + lastbad);
        % Store last good weight.
        lastgood = oldwt;
    else
        % Increase weight.
        if lastgood == -1.0,
            % Simply double.
            newwt = newwt*2.0;
        else
            % Go halfway to last good weight.
            newwt = 0.5*(newwt + lastgood);
        end
        % Store last bad weight.
        lastbad = oldwt;
    end
end

% To be conservative, double the weight before returning it.
newwt = newwt*2.0;
