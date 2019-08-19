% Evaluate the approximate MaxEnt objective.
function [val,grad] = amecost(theta,infos)

% Compute gradient of objective gradient and objective Hessian.
% What we get here is:
% g - the gradient of the reward function with respect to the controls.
% H - the Hessian of the reward function with respect to the controls.
% In the special case of a linear reward, g is simply a linear combination
% of the gradients of various features, so we precompute g for each feature
% and simply linearly combine them here. The same goes for H.
% The objective is 0.5*g'H\g + 0.5*log(|-H|)

val = 0;
grad = zeros(1,size(theta,1));

for i=1:length(infos),
    
    %fprintf(1,'Starting timing.\n');tic;
    
    % Compute g by adding up the gradients of each feature.
    gh = sum(bsxfun(@times,infos{i}.gh,theta'),2);
    g = sum(bsxfun(@times,infos{i}.g,theta'),2);
    %fprintf(1,'Grad: %f\n',toc);tic;
    
    % Compute the block diagonal Ht and Hh.
    Ht = sum(bsxfun(@times,infos{i}.Ht,theta'),2);
    Hh = sum(bsxfun(@times,infos{i}.Hh,theta'),2);
    %fprintf(1,'Hess: %f\n',toc);tic;
    
    % Gradients:
    % dHt = infos{i}.Ht
    % dHh = infos{i}.Hh
    % dg = infos{i}.g
    
    % Solve for h = (J Hh J' + Ht)\g, log determinant, and product traces.
    [h,logDet,Htd,Hhd,Hso] = hesssolve(infos{i},Ht,Hh,g,gh);    
    %fprintf(1,'Inversion: %f\n',toc);tic;

    % Compute objective.
    val = val + 0.5*sum(sum(g.*h,3),1) + 0.5*logDet;
    
    if nargout >= 2,        
        % Compute J'*h.
        Jh = gradprodtr(infos{i}.A,infos{i}.B,h);
        
        % Compute gradient.
        grad = grad + sum(sum(bsxfun(@times,infos{i}.g,h),3),1) - ... % This is h'*dg.
               0.5*sum(sum(sum(bsxfun(@times,bsxfun(@times,infos{i}.Hh,Jh),permute(Jh,[1 2 4 3])),1),3),4) - ... % This is Jh'*dHh*Jh.
               0.5*sum(sum(sum(bsxfun(@times,bsxfun(@times,infos{i}.Ht,h),permute(h,[1 2 4 3])),1),3),4) + ... % This is h'*dHt*h.
               0.5*sum(sum(sum(bsxfun(@times,Htd,infos{i}.Ht),3),4),1) + ... % This is trace(Hinv*dHt)
               0.5*sum(sum(sum(bsxfun(@times,Hhd,infos{i}.Hh),3),4),1) + ... % This is trace([J'HinvJ]*dHh)
               0.5*sum(sum(bsxfun(@times,infos{i}.gh,Hso),3),1); % This is the second order term
    end;
    
    %fprintf(1,'Gradient: %f\n',toc);
end;

% Negate objective and gradient.
val = -val;
grad = -grad';
