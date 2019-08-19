% Quickly solve the JHhatJ'+Htilde system.
function [h,logDet,S,Q,R,X,W] = linearsolve(info,Ht,Hh,g)

% Constants.
Dx = size(Hh,3);
Du = size(Ht,3);
T = size(Ht,1);

% Variables.
Ht = permute(Ht,[3 4 1 2]);
Hh = permute(Hh,[3 4 1 2]);
g = permute(g,[3 2 1]);
hbar = zeros(Dx,1,T);
hbaro = zeros(Dx,1,T);
h = zeros(Du,1,T);
z = zeros(Dx,1,1);
zs = zeros(Dx,Dx);
S = zeros(Dx,Dx,T);
Q = zeros(Dx,Du,T);
R = zeros(Dx,Dx,T);
X = zeros(Dx,Dx,T);
W = zeros(Dx,Dx,T);
logDet = 0;

if Dx == Du,
    % Special upward pass for square matrix case.
    for t=T:-1:1,
        % Compute diagonal entry.
        Dt = info.B(:,:,t)'*(Hh(:,:,t) + zs) + Ht(:,:,t)*info.invB(:,:,t);
        % Invert and compute determinant.
        %if any(any(isnan(Dt))) || any(any(isinf(Dt))),
        %    fprintf(1,'!');
        %end;
        Dinv = inv(Dt);
        Ddet = det(-Dt*info.B(:,:,t));
        if Ddet <= 0.0 || ~isreal(Ddet) || isnan(Ddet) || isinf(Ddet),
            logDet = -Inf;
        else
            logDet = logDet + log(Ddet);
        end;
        % Compute hbar^{(0)}_t
        hbaro(:,:,t) = Dinv*(g(:,:,t) - info.B(:,:,t)'*z);
        % Compute cached matrices for trace computations.
        Q(:,:,t) = Dinv;
        R(:,:,t) = -Dinv*info.B(:,:,t)';
        X(:,:,t) = info.A(:,:,t)'*(Hh(:,:,t) + zs(:,:));
        W(:,:,t) = info.A(:,:,t)';
        % Either express hbar_t in terms of hbar_{t-1}, or solve hbar_1
        if t > 1,
            % Solve for S_t
            S(:,:,t) = Dinv*Ht(:,:,t)*info.invB(:,:,t)*info.A(:,:,t);
            % Update temporary storage.
            z = X(:,:,t)*hbaro(:,:,t) + W(:,:,t)*z;
            zs = X(:,:,t)*S(:,:,t);
        else
            % Find first h entry.
            h(:,:,1) = info.invB(:,:,t)*hbaro(:,:,t);
            % Store corresponding first hbar entry.
            hbar(:,:,1) = hbaro(:,:,t);
        end;
    end;

    % Downward pass.
    for t=2:T,
        % Solve for hbar_t
        hbar(:,:,t) = hbaro(:,:,t) + S(:,:,t)*hbar(:,:,t-1);
        % Solve for h_t using hbar_{t-1}
        h(:,:,t) = info.invB(:,:,t)*(hbar(:,:,t) - info.A(:,:,t)*hbar(:,:,t-1));
    end;
else
    % Additional variables.
    d = zeros(Dx,1,T);
    C = zeros(Dx,Dx,T);
    
    % Upward pass.
    for t=T:-1:1,
        % Compute diagonal entry.
        Dt = info.B(:,:,t)'*(Hh(:,:,t) + zs) + Ht(:,:,t)*info.invB(:,:,t);
        % Add to determinant.
        Ddet = det(-Dt*info.B(:,:,t));
        %if any(any(isnan(Dt))) || any(any(isinf(Dt))),
        %    fprintf(1,'!');
        %end;
        Dinv = pinv(Dt);
        if Ddet <= 0.0 || ~isreal(Ddet) || isnan(Ddet) || isinf(Ddet),
            logDet = -Inf;
        else
            logDet = logDet + log(Ddet);
        end;
        % Compute hbar^{(0)}_t
        hbarzt = Dinv*(g(:,:,t) - info.B(:,:,t)'*z);
        % Compute null space.
        Nt = null(Dt);
        % Compute cached matrices for trace computations.
        invblck = inv([info.B(:,:,t) -Nt]);
        Q(:,:,t) = (eye(Dx) + Nt*invblck((Du+1):end,:))*Dinv;
        R(:,:,t) = -(eye(Dx) + Nt*invblck((Du+1):end,:))*Dinv*info.B(:,:,t)';
        X(:,:,t) = info.A(:,:,t)'*(Hh(:,:,t) + zs(:,:));
        W(:,:,t) = info.A(:,:,t)';
        % Solve for constant term.
        d(:,:,t) = invblck*hbarzt;
        % Compute hbar^{(1)}_t
        hbaro(:,:,t) = hbarzt + Nt*d((Du+1):end,:,t);
        % Either express hbar_t in terms of hbar_{t-1}, or solve hbar_1
        if t > 1,
            % Solve for term dependent on hbar_{t-1}
            C(:,:,t) = invblck*(Dinv*Ht(:,:,t)*info.invB(:,:,t)*info.A(:,:,t) - info.A(:,:,t));
            % Solve for S_t
            S(:,:,t) = Dinv*Ht(:,:,t)*info.invB(:,:,t)*info.A(:,:,t) + Nt*C((Du+1):end,:,t);
            % Update temporary storage.
            z = X(:,:,t)*hbaro(:,:,t) + W(:,:,t)*z;
            zs = X(:,:,t)*S(:,:,t);
        else
            % Find first h entry.
            h(:,:,1) = d(1:Du,:,t);
            % Store corresponding first hbar entry.
            hbar(:,:,1) = hbaro(:,:,t);
        end;
    end;

    % Downward pass.
    for t=2:T,
        % Solve for hbar_t
        hbar(:,:,t) = hbaro(:,:,t) + S(:,:,t)*hbar(:,:,t-1);
        % Solve for h_t using hbar_{t-1}
        h(:,:,t) = d(1:Du,:,t) + C(1:Du,:,t)*hbar(:,:,t-1);
    end;
end;

% Reshape result.
h = permute(h,[3 2 1]);
