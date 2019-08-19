% Evaluate a linear reward function.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    sumevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Evaluate the reward and gradients for all components.
r = zeros(T,1);
if nargout >= 2
    g = zeros(T,Du);
end
if nargout >= 3
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end
if nargout >= 5
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end
if nargout >= 7
    gfull = zeros(T,T*Du);
    Hfull = zeros(T,T*Du,T*Du);
end

% Defer gradient if possible.
mdp_data.defergrad = 1;
deferred = 0;

for i=1:length(reward.theta)
    % Evaluate the components.
    if nargout >= 7
        [cr,cg,cdrdu,cd2rdudu,cdrdx,cd2rdxdx,cgfull,cHfull] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B,dxdu,d2xdudu);
    elseif nargout >= 5
        [cr,cg,cdrdu,cd2rdudu,cdrdx,cd2rdxdx] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    elseif nargout >= 3
        [cr,cg,cdrdu,cd2rdudu] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    elseif nargout >= 2
        [cr,cg] = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states,A,B);
    else
        cr = ...
            feval(strcat(reward.features{i}.type,'evalreward'),...
                  reward.features{i},mdp_data,x,u,states);
    end
    
    % Add up the components.
    r = r + cr*reward.theta(i);
%     if i == 5
%         r = cr*reward.theta(i);
%     end
    if nargout >= 2,
        if iscell(cg),
            if ~deferred,
                gdrdx = zeros(T,Dx);
            end;
            gdrdx = gdrdx + cg{1}*reward.theta(i);
            deferred = 1;
        else
            g = g + cg*reward.theta(i);
        end;
    end;
    if nargout >= 3,
        drdu = drdu + cdrdu*reward.theta(i);
        d2rdudu = d2rdudu + cd2rdudu*reward.theta(i);
    end;
    if nargout >= 5,
        drdx = drdx + cdrdx*reward.theta(i);
        d2rdxdx = d2rdxdx + cd2rdxdx*reward.theta(i);
    end;
    if nargout >= 7,
        gfull = gfull + cgfull*reward.theta(i);
        Hfull = Hfull + cHfull*reward.theta(i);
    end;
end;

if nargout >= 2 && deferred,
    % Add deferred gradients.
    g = g + permute(gradprod(A,B,permute(gdrdx,[1 3 2])),[1 3 2]);
end;
