% Master Hessian solver.
function [h,logDet,Htd,Hhd,Hso] = hesssolve(info,Ht,Hh,g,gh)

% Get constants.
T = size(info.Ht,1);
Du = size(info.Ht,3);
Dx = size(info.Hh,3);

if info.linear_dynamics,
    % Initialize Hso to zero (no second order term).
    Hso = zeros(T,1,Dx);
    
    %fprintf(1,'Timing:\n');tic;
    
    % Perform linear solve to get h, logDet, and auxiliaries.
    [h,logDet,S,Q,R,X,W] = linearsolve(info,Ht,Hh,g);
    
    %fprintf(1,'Solve: %f\n',toc);tic;
    
    % Solve for trace terms.
    Htd = tracetilde(info,S,Q,R,X,W);
    
    %fprintf(1,'Traces A: %f\n',toc);tic;
    
    Hhd = tracehat(info,S,Q,R,X,W);
    
    %fprintf(1,'Traces: %f\n',toc);tic;
else
    % Using nonlinear version - do it the slow way.
    % Reconstruct J.
    J = zeros(T*Du,T*Dx);
    for d=1:Dx,
        J(:,(d-1)*T + (1:T)) = info.J(:,:,d)';
    end;

    % Form the complete huge Hessian.
    Htfull = zeros(Du*T,Du*T);
    Hhfull = zeros(Dx*T,Dx*T);
    for t=1:T,
        idxu = t + (0:(Du-1))*T;
        idxx = t + (0:(Dx-1))*T;
        Htfull(idxu,idxu) = Ht(t,1,:,:);
        Hhfull(idxx,idxx) = Hh(t,1,:,:);
    end;
    H = J*Hhfull*J' + Htfull + permute(sum(sum(bsxfun(@times,info.JJ,permute(gh,[1 2 4 3])),1),4),[2 3 1 4]);
    
    % Solve the linear system.
    [L,p] = chol(-H,'lower');
    if p == 0,
        % All is well - matrix is SPD.
        invH = -L'\(L\eye(T*Du));
        logDet = 2.0*sum(log(diag(L)));
    else
        invH = inv(H);
        logDet = -Inf;
    end;

    % Return h.
    h = invH*g(:);
    h = reshape(h,[T 1 Du]);

    % Compute traces.
    if nargout >= 3,
        Htd = zeros(T,1,Du,Du);
        Hhd = zeros(T,1,Dx,Dx);
        JpinvHJ = J'*invH*J;
        for t=1:T,
            idxu = t + (0:(Du-1))*T;
            idxx = t + (0:(Dx-1))*T;
            Htd(t,1,:,:) = invH(idxu,idxu);
            Hhd(t,1,:,:) = JpinvHJ(idxx,idxx);
        end;
    end;

    % Compute second order term.
    if nargout >= 5,
        Hso = permute(sum(sum(bsxfun(@times,permute(invH-h(:)*h(:)',[3 4 1 2]),permute(info.JJ,[1 4 2 3])),3),4),[1 3 2]);
    end;
end;
