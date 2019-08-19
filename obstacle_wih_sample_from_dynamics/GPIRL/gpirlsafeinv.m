% Safe inversion of kernel matrix that uses SVD decomposition if Cholesky
% fails.
function [halfLogDet,invK,alpha] = gpirlsafeinv(K,u)

% First, try Cholesky.
[L,p] = chol(K,'lower');
if p == 0,
    halfLogDet = sum(log(diag(L)));
    invK = L'\(L\eye(size(K,1)));
    if nargin > 1,
        alpha = L'\(L\u);
    end;
else
    % Must do full SVD decomposition.
    warning('Cholesky failed, switching to SVD');
    [U,S,V] = svd(K);
    dS = diag(S);
    Sinv = diag(1./dS);
    halfLogDet = 0.5*sum(log(dS));
    invK = V*Sinv*U';
    if nargin > 1,
        alpha = invK*u;
    end;
end;
