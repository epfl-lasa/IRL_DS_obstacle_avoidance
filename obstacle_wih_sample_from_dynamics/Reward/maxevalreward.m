% Evaluate a maximum or minimum of the component features.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    maxevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Evaluate the reward and gradients for all components.
if reward.min,
    r = Inf*ones(T,1);
else
    r = -Inf*ones(T,1);
end;
if nargout >= 2,
    gdrdx = zeros(T,Dx);
end;
if nargout >= 3,
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end;
if nargout >= 5,
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end;
if nargout >= 7,
    gfull = zeros(T,T*Du);
    Hfull = zeros(T,T*Du,T*Du);
end;

% Defer gradient if possible.
mdp_data.defergrad = 1;
for i=1:length(reward.features),
    % Evaluate the components.
    if nargout >= 5,
        [cr,cg,cdrdu,cd2rdudu,cdrdx,cd2rdxdx] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    elseif nargout >= 3,
        [cr,cg,cdrdu,cd2rdudu] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    elseif nargout >= 2,
        [cr,cg] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    else
        cr = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states);
    end;
    
    % Take the best components.
    if reward.min,
        bestidx = cr < r;
    else
        bestidx = cr > r;
    end;
    r(bestidx) = cr(bestidx);
    if nargout >= 2,
        if ~iscell(cg),
            error('Max reward only supports deferred gradient!');
        end;
        gdrdx(bestidx,:) = cg{1}(bestidx,:);
    end;
    if nargout >= 3,
        drdu(bestidx,:) = cdrdu(bestidx,:);
        d2rdudu(bestidx,:,:) = cd2rdudu(bestidx,:,:);
    end;
    if nargout >= 5,
        drdx(bestidx,:) = cdrdx(bestidx,:);
        d2rdxdx(bestidx,:,:) = cd2rdxdx(bestidx,:,:);
    end;
end;

if nargout >= 2,
    % Compute deferred gradients.
    g = permute(gradprod(A,B,permute(gdrdx,[1 3 2])),[1 3 2]);
end;

if nargout >= 7,
    % Compute gfull.
    % Convert gradient to T x TD matrix.
    drdxmat = zeros(T,T*Dx);
    for i=1:Dx,
        drdxmat(:,(i-1)*T + (1:T)) = diag(drdx(:,i));
    end;

    % Compute gradient with respect to controls.
    gfull = drdxmat * dxdu';
    
    % Compute Hfull.
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T,
        idxs = (0:(Dx-1))*T + t;
        D = permute(d2rdxdx(t,:,:),[2 3 1]);
        Hfull(t,:,:) = dxdu(:,idxs) * D * dxdu(:,idxs)' + sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end;
end;
