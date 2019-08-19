function options = check_options(varargin)
if ~isempty(varargin)
    options = varargin{1};
else
    options.dt=0.02; %to create the variable
end

if ~isfield(options,'dt') % integration time step
    options.dt = 0.02;
end
if ~isfield(options,'model') % first order ordinary differential equation
    options.model = 1;
end
if ~isfield(options,'i_max') % maximum number of iterations
    options.i_max = 1000;
end
if ~isfield(options,'tol') % convergence tolerance
    options.tol = 0.001;
end
if ~isfield(options,'plot') % shall simulator plot the figure
    options.plot = 1;
else 
    options.plot = options.plot > 0;
end
if ~isfield(options,'timeDependent') % shall simulator plot the figure
    options.timeDependent = 0;
else 
    options.timeDependent = options.timeDependent > 0;
end
if ~isfield(options,'perturbation') % shall TODO?
    options.perturbation.type = '';
else 
    if ~isfield(options.perturbation,'type') || ~isfield(options.perturbation,'t0') || ~isfield(options.perturbation,'dx') || ...
        ((strcmpi(options.perturbation.type,'rcp') || strcmpi(options.perturbation.type,'tcp')) && ~isfield(options.perturbation,'tf')) || ...
        (~strcmpi(options.perturbation.type,'rcp') && ~strcmpi(options.perturbation.type,'tcp') && ~strcmpi(options.perturbation.type,'rdp') && ~strcmpi(options.perturbation.type,'tdp'))
    
        disp('Invalid perturbation structure. The perturbation input is ignored!')
        options.perturbation.type = '';
    end
end