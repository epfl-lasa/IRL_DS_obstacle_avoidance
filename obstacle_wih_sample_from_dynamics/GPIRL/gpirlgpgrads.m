% Compute GP term gradient, Hessian, and gradients.
function [K,g,gh,gt,Hh,Ht,h,logDet,Htd,Hhd,Jh,Hso,ugrad,iwgrad,rbfgrad] = ...
    gpirlgpgrads(glin,invK_uu,alpha,info,gp,Hhlin,Htlin,dalphadiw)

%fprintf(1,'Starting timing.\n');tic;

% Constants.
T = size(info.f,1);
F = size(info.f,2);
Du = size(info.gt,3);
Dx = size(info.gh,3);
N = size(gp.F_u,1);

% Get gp parameters.
inv_widths = gpirlhpxform(gp.inv_widths,[],gp.ard_xform,1); % This is \Lambda
noise_var = gpirlhpxform(gp.noise_var,[],gp.noise_xform,1); % This is 2\siddhsa^2
rbf_var = gpirlhpxform(gp.rbf_var,[],gp.rbf_xform,1); % This is \beta
inv_widths = min(inv_widths,1e50); % Prevent overflow.
rbf_var = min(rbf_var,1e50); % Prevent overflow.
iw_sqrt = sqrt(inv_widths);

% Construct the RBF kernel.
nconst = exp(-0.5*noise_var*sum(inv_widths)); % This is uniform position noise.
sXpaths = bsxfun(@times,info.f,iw_sqrt);
sF_u = bsxfun(@times,gp.F_u,iw_sqrt);
dists = max(bsxfun(@plus,sum(sXpaths.^2,2),sum(sF_u.^2,2)') - 2*(sXpaths*(sF_u')),0);
K = nconst*rbf_var*exp(-0.5*dists);

%fprintf(1,'Kernel: %f\n',toc);tic;

% Stop here if we only want the kernel.
if nargout <= 1,
    return;
end;

% Construct gradient.
% Precompute K * alpha.
Ka = bsxfun(@times,K,alpha');

% Construct delta matrix.
delta = bsxfun(@minus,permute(gp.F_u,[3 1 2]),permute(info.f,[1 3 2]));

% Precompute lambda * delta * K * alpha for each feature.
lamdKa = bsxfun(@times,inv_widths,permute(sum(bsxfun(@times,delta,Ka),2),[1 3 2]));
%lamdKa = zeros(T,F);
%for k=1:F,
%    lamdKa(:,k) = inv_widths(k)*sum(bsxfun(@minus,gp.F_u(:,k)',info.f(:,k)).*Ka,2);
%end;

% Compute gradient: g_t = jac * sum_k gh_t sum_i lam_k del_tik Ka_ti
gh = sum(bsxfun(@times,info.gh,lamdKa),2);
g = glin + gradprod(info.A,info.B,gh);
gt = glin;

%fprintf(1,'Grad: %f\n',toc);tic;

% Stop here if we only want the kernel and gradient.
if nargout <= 4,
    return;
end;

% Construct Hessian.
% The first component: Hh_tpq = sum_k lam_k Hh_tkpq sum_i delta_tik Ka_ti
Hh = Hhlin + sum(bsxfun(@times,info.Hh,lamdKa),2);

%fprintf(1,'H1: %f\n',toc);tic;

% The second component: H_tpq -= sum_k lam_k gh_tkp gh_tkq sum_i Ka_ti
Hh = Hh - bsxfun(@times,sum(bsxfun(@times,inv_widths,bsxfun(@times,info.gh,permute(info.gh,[1 2 4 3]))),2),sum(Ka,2));

%fprintf(1,'H2: %f\n',toc);tic;

% The third component.
% First compute using state gradients as ddhs = sum_k gh_tkj lam_k del_tik
ddhs = sum(bsxfun(@times,permute(info.gh,[1 4 3 2]),bsxfun(@times,permute(inv_widths,[1 3 4 2]),permute(delta,[1 2 4 3]))),4);
% Now add H_tpq += sum_i ddhs_tip ddhs_tiq Ka_ti
Hh = Hh + sum(bsxfun(@times,bsxfun(@times,ddhs,permute(ddhs,[1 2 4 3])),Ka),2);

%fprintf(1,'H3: %f\n',toc);tic;

% Set Ht
Ht = Htlin;

% Stop here if we only want the kernel, gradient and Hessian.
if nargout <= 6,
    return;
end;

% Compute h and trace entries.
if nargout <= 8,
    % Don't include trace entries.
    [h,logDet] = hesssolve(info,Ht,Hh,g,gh);
elseif nargout <= 10,
    % Don't include second order term.
    [h,logDet,Htd,Hhd] = hesssolve(info,Ht,Hh,g,gh);
else
    % Include everything.
    [h,logDet,Htd,Hhd,Hso] = hesssolve(info,Ht,Hh,g,gh);
end;

%fprintf(1,'Inv: %f\n',toc);tic;

% Stop here if we only want the kernel, gradient, Hessian, and solved linear system.
if nargout <= 10,
    return;
end;

% Multiply h by Jacobian transpose.
Jh = gradprodtr(info.A,info.B,h);

% Stop here if we only want Jh in addition.
if nargout <= 12,
    return;
end;

% Compute first order difference matrix Hfo_tij = [J'H^{-1}J]_tij - [J'h]_ti[J'h]_tj
Hfo = Hhd - bsxfun(@times,Jh,permute(Jh,[1 2 4 3]));

% dbar = sum_i Jh_ti dgh_tki
dbar = sum(bsxfun(@times,Jh,info.gh),3);

% Dbar = sum_ij Hfo_tij dHhat_tkij + sum_i Hso_ti dghat_tki
Dbar = sum(sum(bsxfun(@times,Hfo,info.Hh),3),4) + sum(bsxfun(@times,Hso,info.gh),3);

% ddbar = sum_ij Hfo_tij dghat_tki dghat_tkj
ddbar = sum(bsxfun(@times,sum(bsxfun(@times,Hfo,info.gh),3),permute(info.gh,[1 2 4 3])),4);

%fprintf(1,'Bars: %f\n',toc);tic;

% Precompute shared matrix for gradients.
%gradmat = zeros(T,N);
%for k=1:F,
%    % These are the linear dependence terms.
%    gradmat = gradmat + inv_widths(k)*bsxfun(@minus,...
%        bsxfun(@times,dbar(:,k)+0.5*Dbar(:,k),bsxfun(@minus,gp.F_u(:,k)',info.f(:,k))),...
%        0.5*ddbar(:,k));
%end;
gradmat = sum(bsxfun(@times,permute(inv_widths,[1 3 2]),bsxfun(@minus,bsxfun(@times,...
    permute(dbar + 0.5*Dbar,[1 3 2]),delta),permute(0.5*ddbar,[1 3 2]))),3);

% Now add sum_j1j2 Hfo_j1j2 ddhs_tij1 ddhs_tij2 to gradmat
gradmat = gradmat + sum(sum(bsxfun(@times,0.5*Hfo,bsxfun(@times,ddhs,permute(ddhs,[1 2 4 3]))),3),4);

% Multiply by K.
gradmat = gradmat.*K;

%fprintf(1,'Gradmat: %f\n',toc);tic;

% Compute gradient with respect to u.
ugrad = (sum(gradmat,1)*invK_uu)';

%fprintf(1,'Umat: %f\n',toc);tic;

% Compute gradient with respect to inverse widths.
iwgrad = zeros(1,F);
if gp.learn_ard,
    iwgrad = sum(bsxfun(@times,sum(gradmat,1)',dalphadiw),1) - ...
             0.5*permute(sum(bsxfun(@times,sum(bsxfun(@times,gradmat,delta.^2 + noise_var),1),alpha'),2),[1 3 2]) + ...
             permute(sum(bsxfun(@times,sum(bsxfun(@times,permute(dbar,[1 3 2]),bsxfun(@times,delta,K)),1),alpha'),2),[1 3 2]) + ...
             0.5*permute(sum(bsxfun(@times,sum(bsxfun(@times,(bsxfun(@minus,bsxfun(@times,permute(Dbar,[1 3 2]),delta),permute(ddbar,[1 3 2])) + ...
                      2.0*bsxfun(@times,delta,...
                                 permute(sum(bsxfun(@times,ddhs,permute(sum(bsxfun(@times,Hfo,permute(info.gh,[1 2 4 3])),4),[1 4 3 2])),3),[1 2 4 3])...
                                 )),K),1),alpha'),2),[1 3 2]);
    %{
    for k=1:F,        
        % Precompute summat = Hfo * ddhs * dXdS_k^T34.
        summat = sum(sum(bsxfun(@times,Hfo,bsxfun(@times,ddhs,permute(info.gh(:,k,:),[1 2 4 3]))),3),4);
        
        % Compute gradient.
        iwgrad(k) = sum(gradmat,1)*dalphadiw(:,k) - 0.5*sum(gradmat.*(delta(:,:,k).^2 + noise_var),1)*alpha + ...
                    sum(bsxfun(@times,dbar(:,k),delta(:,:,k).*K),1)*alpha + ...
                    0.5*sum((bsxfun(@minus,bsxfun(@times,Dbar(:,k),delta(:,:,k)),ddbar(:,k)) + 2.0*delta(:,:,k).*summat).*K,1)*alpha;
    end;
    %}
end;

%fprintf(1,'Iw: %f\n',toc);tic;

% Gradient of rbf variance is always zero here, because K and invK_uu cause
% rbf variance to cancel out.
rbfgrad = 0;
