% Compute MaxEnt objective and gradient. Fine horizon version.
function [val,dr] = maxentcost(r,F,muE,mu_sa,mdp_data,initD,T)

% Compute constants.
[states,actions,transitions] = size(mdp_data.sa_p);

% Convert to full reward.
wts = r;
r = sum(bsxfun(@times,F,permute(r,[2 3 1])),3);

% Use dynamic programming to compute the full value function.
value_function = zeros(states,T);
valrow = maxentsoftmax(r);
value_function(:,T) = valrow;
for t=(T-1):-1:1,
    valrow = maxentsoftmax(...
        r + sum(mdp_data.sa_p.*valrow(mdp_data.sa_s),3));
    value_function(:,t) = valrow;
end;

% Compute log policy.
logpolicy = zeros(states,actions,T);
for t=1:T,
    if t < T,
        valrow = value_function(:,t+1);
        q = r + sum(mdp_data.sa_p.*valrow(mdp_data.sa_s),3);
    else
        q = r;
    end;
    logpolicy(:,:,t) = q - repmat(value_function(:,t),1,actions);
end;

% Compute value by adding up log example probabilities.
val = sum(sum(sum(logpolicy.*mu_sa)));

% Invert for descent.
val = -val;

if nargout >= 2,
    %{
    dVdT = zeros(states,size(F,3));
    dr2 = zeros(size(F,3),1);
    for t=T:-1:1,
        dVdt = permute(sum(bsxfun(@times,exp(logpolicy(:,:,t)),(F + ...
            permute(...
                sum(reshape(bsxfun(@times,dVdT(mdp_data.sa_s,:),mdp_data.sa_p(:)),[states actions transitions size(F,3)]),3),...
                [1 2 4 3]))),2),[1 3 2]);
        dVdtpo = permute(sum(bsxfun(@times,mu_sa(:,:,t),...
            permute(...
                sum(reshape(bsxfun(@times,dVdT(mdp_data.sa_s,:),mdp_data.sa_p(:)),[states actions transitions size(F,3)]),3),...
                [1 2 4 3])),2),[1 3 2]);
        dr2 = dr2 + muE(:,t) + sum(dVdtpo,1)' - sum(bsxfun(@times,sum(mu_sa(:,:,t),2),dVdt),1)';
        dVdT = dVdt;
    end;
    %}
    
    % Compute state-action visitation count D along with the gradient.
    Dp = zeros(states,1);
    dr = zeros(size(F,3),1);
    for t=1:T,
        % First compute the state visition count for this time step.
        Ds = initD(:,:,t) + Dp;
        
        % Now compute action probabilities.
        D = bsxfun(@times,Ds,exp(logpolicy(:,:,t)));
        dr = dr + muE(:,t) - permute(sum(sum(bsxfun(@times,F,D),1),2),[3 1 2]);
        
        % Now compute a new Dp using transition probabilities.
        Dpi = mdp_data.sa_p.*repmat(D,[1 1 transitions]);
        Dp = sum(sparse(mdp_data.sa_s(:),1:states*actions*transitions,...
            Dpi(:),states,states*actions*transitions),2);
    end;
    
    % Invert for descent.
    dr = -dr;
end;
