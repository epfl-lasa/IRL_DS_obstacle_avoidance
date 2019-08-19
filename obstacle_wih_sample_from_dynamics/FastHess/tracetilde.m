% Get diagonal terms of H^{-1}
function Htd = tracetilde(info,S,Q,R,X,W)

% Constants.
Dx = size(Q,1);
Du = size(Q,2);
T = size(Q,3);

% Variables.
Htd = zeros(Du,Du,T);

% Compute first step.
Htd(:,:,1) = info.invB(:,:,1)*Q(:,:,1);
V = R(:,:,1);

% Compute remaining steps.
for t=2:T,
    Htd(:,:,t) = info.invB(:,:,t)*(eye(Dx) + (S(:,:,t) - info.A(:,:,t))*V*X(:,:,t))*Q(:,:,t);
    V = S(:,:,t)*V*(X(:,:,t)*R(:,:,t) + W(:,:,t)) + R(:,:,t);
end;

% Reshape result.
Htd = permute(Htd,[3 4 1 2]);
