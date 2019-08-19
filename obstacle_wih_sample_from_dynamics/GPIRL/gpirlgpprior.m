% Compute GP prior term and its gradients.
function [K_uu,invK_uu,alpha,ll,ugrad,iwgrad,rbfgrad,dalphadiw] = gpirlgpprior(gp,u)

% Constants.
N = size(gp.F_u,1);
F = size(gp.F_u,2);

% Undo transforms.
inv_widths = gpirlhpxform(gp.inv_widths,[],gp.ard_xform,1); % This is \Lambda
noise_var = gpirlhpxform(gp.noise_var,[],gp.noise_xform,1); % This is 2\sigma^2
rbf_var = gpirlhpxform(gp.rbf_var,[],gp.rbf_xform,1); % This is \beta
inv_widths = min(inv_widths,1e50); % Prevent overflow.
rbf_var = min(rbf_var,1e50); % Prevent overflow.

% Compute scales.
iw_sqrt = sqrt(inv_widths);

% Scale positions in feature space.
sF_u = bsxfun(@times,iw_sqrt,gp.F_u);

% Construct noise matrix.
mask_mat = ones(N)-eye(N);
nconst = exp(-0.5*noise_var*sum(inv_widths));
nmat = nconst*ones(N) + (1-nconst)*eye(N);

% Compute K_uu matrix.f
d_uu = bsxfun(@plus,sum(sF_u.^2,2),sum(sF_u.^2,2)') - 2*(sF_u*(sF_u'));
d_uu = max(d_uu,0);
d_uu = d_uu.*mask_mat; % Mask out values on the diagonal in case of numerical instability.
K_uu = rbf_var*exp(-0.5*d_uu).*nmat;

% Invert the kernel matrix.
[halfLogDet,invK_uu,alpha] = gpirlsafeinv(K_uu,u);

if nargout > 3
    % Compute likelihood.
    ll = - 0.5*u'*alpha - halfLogDet - 0.5*sum(sum(invK_uu.^2));
    
    % Add priors.
    if gp.learn_ard
        ll = ll + gpirlhpprior(gp.inv_widths,gp.ard_prior,gp.ard_prior_wt,gp.ard_xform,gp);
    end
    if gp.learn_rbf
        ll = ll + gpirlhpprior(gp.rbf_var,gp.rbf_prior,gp.rbf_prior_wt,gp.rbf_xform,gp);
    end
end

% Compute dalphas and gradients.
if nargout > 4
    % Compute gradient of u.
    ugrad = - alpha;

    % Precompute difference matrix.
    dm = 0.5*4*(invK_uu^3) + alpha*alpha' - invK_uu;

    % Compute gradient with respect to each inverse width.
    if gp.learn_ard
        % Allocate gradients.
        dalphadiw = zeros(size(alpha,1),F);
        iwgrad = zeros(1,F);

        for f=1:F
            % Compute gradient of kernel matrix.
            dKdiw = -0.5*((bsxfun(@minus,gp.F_u(:,f)',gp.F_u(:,f)).^2) + noise_var*mask_mat).*K_uu;

            % Compute gradient of alpha.
            dalphadiw(:,f) = -invK_uu*(dKdiw*alpha);

            % Compute likelihood gradient.
            iwgrad(f) = 0.5*sum(sum(dm.*dKdiw,1),2);
        end
        
        % Transform.
        iwgrad = gpirlhpxform(gp.inv_widths,iwgrad,gp.ard_xform,2);
    else
        dalphadiw = 0;
        iwgrad = 0;
    end

    % Compute gradient with respect to rbf variance.
    if gp.learn_ard
        dalphadrbf = -alpha/rbf_var;
        rbfgrad = 0.5*sum(sum(dm.*K_uu/rbf_var,1),2);
        rbfgrad = gpirlhpxform(gp.rbf_var,rbfgrad,gp.rbf_xform,2);
    else
        dalphadrbf = 0;
        rbfgrad = 0;
    end

    % Compute hyperparameter prior gradients.
    if gp.learn_ard
        iwgrad = iwgrad + gpirlhppriorgrad(gp.inv_widths,gp.ard_prior,gp.ard_prior_wt,gp.ard_xform,gp);
    end
    if gp.learn_rbf
        rbfgrad = rbfgrad + gpirlhppriorgrad(gp.rbf_var,gp.rbf_prior,gp.rbf_prior_wt,gp.rbf_xform,gp);
    end
end
