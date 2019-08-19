% Place parameters from parameter vector into GP.
function [gp,u,wts] = gpirlunpack(gp,x)

% Count the last index read.
endi = length(x);

% Read RBF variance hyperparameter if we are learning it.
if gp.learn_rbf
    gp.rbf_var = x(endi);
    endi = endi-1;
end

% Read noise hyperparameter if we are learning it.
if gp.learn_noise
    gp.noise_var = x(endi);
    endi = endi-1;
end

% Read ARD kernel parameters.
if gp.learn_ard
    gp.inv_widths = x(endi-length(gp.inv_widths)+1:endi)';
    endi = endi-length(gp.inv_widths);
end

% Read reward parameters.
u = x((endi-size(gp.F_u,1)+1):endi);
endi = endi-size(gp.F_u,1);
wts = x(1:endi);
