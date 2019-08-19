% Get diagonal terms of J'*H^{-1}*J
function Hhd = tracehat(info,S,Q,R,X,W)

% Constants.
Dx = size(Q,1);
T = size(Q,3);

% Variables.
Hhd = zeros(Dx,Dx,T);

% Fill in the first one.
Hhd(:,:,1) = info.B(:,:,1)*info.invB(:,:,1)*Q(:,:,1)*info.B(:,:,1)';
hbartt = Q(:,:,1)*info.B(:,:,1)';
Vtt = R(:,:,1);
Ft = info.B(:,:,1)*info.invB(:,:,1)*Vtt;

for t=2:T,
    % Precompute and store values.
    U = X(:,:,t)*R(:,:,t) + W(:,:,t);
    XQBt = X(:,:,t)*Q(:,:,t)*info.B(:,:,t)';
    QBt = Q(:,:,t)*info.B(:,:,t)';
    BBinv = info.B(:,:,t)*info.invB(:,:,t);
    VttU = Vtt*U;
    temph = hbartt*info.A(:,:,t)' + Vtt*XQBt;
    
    % Perform recursive step.
    Hhd(:,:,t) = info.A(:,:,t)*(Hhd(:,:,t-1)*info.A(:,:,t)' + Ft*XQBt) + ...
        BBinv*(QBt + (S(:,:,t) - info.A(:,:,t))*temph);
    hbartt = QBt + S(:,:,t)*temph;
    Vtt = R(:,:,t) + S(:,:,t)*VttU;
    Ft = info.A(:,:,t)*Ft*U + BBinv*(Vtt - info.A(:,:,t)*VttU);
end;

% Reshape result.
Hhd = permute(Hhd,[3 4 1 2]);
