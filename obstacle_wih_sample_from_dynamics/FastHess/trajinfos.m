% Compute trajectory info structs for each trajectory.
function infos = trajinfos(features,mdp,mdp_data,example_samples)

% Compute constants.
N = length(example_samples);
F = length(features);

% Allocate trajectory infos.
infos = cell(1,N);
for i=1:N
    % Get controls and initial state.
    x = example_samples{i}.s;
    u = example_samples{i}.u;
    [states,A,B,invB,dxdu,d2xdudu] = feval(strcat(mdp,'control'),mdp_data,x,u);
    T = size(u,1);
    Du = size(u,2);
    Dx = size(states,2);
    
    % Create structure.
    infos{i} = struct(...
        'linear_dynamics',sum(sum(sum(abs(d2xdudu)))) <= 1.0e-8,...
        'f',zeros(T,F),...
        'g',zeros(T,F,Du),...
        'gh',zeros(T,F,Dx),...
        'dt',zeros(T,F,Du),...
        'Hh',zeros(T,F,Dx,Dx),...
        'Ht',zeros(T,F,Du,Du));
    
    % Choose whether to store A/B matrices or J/JJ.
    if ~infos{i}.linear_dynamics
        % Build the state Jacobian and Hessian.
        infos{i}.J = zeros(T,T*Du,Dx);
        infos{i}.JJ = zeros(T,T*Du,T*Du,Dx);
        for d=1:Dx
            infos{i}.J(:,:,d) = dxdu(:,(d-1)*T + (1:T))';
            infos{i}.JJ(:,:,:,d) = permute(d2xdudu(:,:,(d-1)*T + (1:T)),[3 1 2]);
        end
        % Allocate full gradients and Hessians for nonlinear dynamics.
        infos{i}.gfull = zeros(T,F,T*Du);
        infos{i}.Hfull = zeros(T,F,T*Du,T*Du);
    end
    
    % Simply store A and B matrices.
    infos{i}.A = A;
    infos{i}.B = B;
    infos{i}.invB = invB;
    
    for f=1:F
        % Compute the gradient and Hessian of feature.
        if ~infos{i}.linear_dynamics
            [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
                feval(strcat(features{f}.type,'evalreward'),...
                    features{f},mdp_data,x,u,states,A,B,dxdu,d2xdudu);
        else
            [r,g,drdu,d2rdudu,drdx,d2rdxdx] = ...
                feval(strcat(features{f}.type,'evalreward'),...
                    features{f},mdp_data,x,u,states,A,B);
        end
        
        % Check Hessian if desired.
        %if ~strcmp(features{f}.type,'reg'),
        %	fprintf(1,'Feature %s\n',features{f}.type);
        %    checkhessian(@(u)checkhessianfun(features{f},mdp,mdp_data,x,u,T),u(:));
        %end;
        
        % Save value.
        infos{i}.f(:,f) = r;
        
        % Save gradients.
        infos{i}.g(:,f,:) = permute(g,[1 3 2]);
        infos{i}.gh(:,f,:) = permute(drdx,[1 3 2]);
        infos{i}.gt(:,f,:) = permute(drdu,[1 3 2]);
        infos{i}.Hh(:,f,:,:) = permute(d2rdxdx,[1 4 2 3]);
        infos{i}.Ht(:,f,:,:) = permute(d2rdudu,[1 4 2 3]);
        
        % Save full gradients if desired.
        if ~infos{i}.linear_dynamics
            infos{i}.gfull(:,f,:) = permute(gfull,[1 3 2]);
            infos{i}.Hfull(:,f,:,:) = permute(Hfull,[1 4 2 3]);
        end
    end
end
