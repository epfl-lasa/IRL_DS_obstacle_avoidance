% Check to make sure the Hessian is correct.
function checkhessian(fun,pt)

% Compute finite differences grad.
[~,g] = autoGrad(pt,0,fun);
[~,ag,aH] = fun(pt);

% Compute finite differences Hessian.
[~,~,H] = autoHess(pt,0,fun);

% Check maximum gradient difference.
maxGrad = max(abs(g(:)-ag(:)));
maxHess = max(abs(H(:)-aH(:)));

fprintf(1,'Max grad mismatch: %f\n',maxGrad);
if maxGrad > 1.0e-4,
    disp([g(:) ag(:)]);
end;

fprintf(1,'Max Hessian mismatch: %f\n',maxHess);
if maxHess > 1.0e-4,
    disp([H(:) aH(:)]);
end;
