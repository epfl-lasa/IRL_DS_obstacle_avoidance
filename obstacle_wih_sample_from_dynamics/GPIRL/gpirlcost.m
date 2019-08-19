% Evaluate the approximate GPIRL objective.
function [ll,llgrad] = gpirlcost(params,gp,infos_dyn,infos_pt)

% Unpack parameters.
[gp,u,wts] = gpirlunpack(gp,params);

% Initialize gradient.
gradgp = gp;
gradgp.inv_widths = zeros(size(gradgp.inv_widths));
gradgp.rbf_var = zeros(size(gradgp.rbf_var));
gradgp.noise_var = zeros(size(gradgp.noise_var));
gradu = zeros(size(u));
gradwts = zeros(1,size(wts,1));

% Compute GP prior term and its gradients.
[~,invK_uu,alpha,ll,ugrad,iwgrad,rbfgrad,dalphadiw] = ...
    gpirlgpprior(gp,u);

% Add to gradients.
gradu = gradu + ugrad;
if gp.learn_ard
    gradgp.inv_widths = gradgp.inv_widths + iwgrad;
end
if gp.learn_rbf
    gradgp.rbf_var = gradgp.rbf_var + rbfgrad;
end

% Compute priors on linear terms.
for i=1:length(gp.dyn_priors)
    ll = ll + gpirlhpprior(wts(i),gp.dyn_priors{i},gp.dyn_prior_wts(i),'none',gp);
    gradwts(1,i,1) = gradwts(1,i,1) + gpirlhppriorgrad(wts(i),gp.dyn_priors{i},gp.dyn_prior_wts(i),'none',gp);
end

%%%%%%%%%%%%
% discounting factor [failed]
discount = 1;
%%%%%%%%%%%%

% Step over each trajectory.
for i=1:length(infos_pt)
    % Compute gradient and Hessian contributions from linear entries.
    glin = sum(bsxfun(@times,infos_dyn{i}.g,wts'),2);
    Htlin = sum(bsxfun(@times,infos_dyn{i}.Ht,wts'),2);
    Hhlin = sum(bsxfun(@times,infos_dyn{i}.Hh,wts'),2);
    
    % Compute GP term gradient, Hessian, and gradients.
    if nargout > 1
        [~,g,~,~,~,~,h,logDet,Htd,Hhd,Jh,Hso,ugrad,iwgrad,rbfgrad] = ...
            gpirlgpgrads(glin,invK_uu,alpha,infos_pt{i},gp,Hhlin,Htlin,dalphadiw);
    else
        [~,g,~,~,~,~,h,logDet] = ...
            gpirlgpgrads(glin,invK_uu,alpha,infos_pt{i},gp,Hhlin,Htlin);
    end
    
    % Add to likelihood.
    ll = discount*ll + 0.5*sum(sum(g.*h,3),1)*infos_pt{i}.discount + 0.5*logDet*infos_pt{i}.discount;
    
    if nargout > 1
        % Compute gradient with respect to linear terms.
        gradwts = discount*gradwts + (sum(sum(bsxfun(@times,infos_dyn{i}.g,h),3),1) - ... % This is h'*dg.
               0.5*sum(sum(sum(bsxfun(@times,bsxfun(@times,infos_dyn{i}.Hh,Jh),permute(Jh,[1 2 4 3])),1),3),4) - ... % This is Jh'*dHh*Jh.
               0.5*sum(sum(sum(bsxfun(@times,bsxfun(@times,infos_dyn{i}.Ht,h),permute(h,[1 2 4 3])),1),3),4) + ... % This is h'*dHt*h.
               0.5*sum(sum(sum(bsxfun(@times,Htd,infos_dyn{i}.Ht),3),4),1) + ... % This is trace(Hinv*dHt)
               0.5*sum(sum(sum(bsxfun(@times,Hhd,infos_dyn{i}.Hh),3),4),1) + ... % This is trace([J'HinvJ]*dHh)
               0.5*sum(sum(bsxfun(@times,infos_dyn{i}.gh,Hso),3),1)) ...  % This is the second order term
                *infos_pt{i}.discount; 

        % Add to gradients.
        gradu = gradu + ugrad*infos_pt{i}.discount;
        if gp.learn_ard
            gradgp.inv_widths = gradgp.inv_widths + gpirlhpxform(gp.inv_widths,iwgrad,gp.ard_xform,2)*infos_pt{i}.discount;
        end
        if gp.learn_rbf
            gradgp.rbf_var = gradgp.rbf_var + gpirlhpxform(gp.rbf_var,rbfgrad,gp.rbf_xform,2)*infos_pt{i}.discount;
        end
    end
end

% Negate objective and gradient.
ll = -ll;
llgrad = -gpirlpack(gradgp,gradu,gradwts');
