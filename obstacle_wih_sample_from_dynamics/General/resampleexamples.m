% Resample example tranjectories from the state space of a given MDP using a new reward.
function [example_samples,test_samples, reward] = resampleexamples(mdp_data, mdp, reward,...
                            true_reward, test_params, old_examples, old_tests, verbose)

pllot = 0;
                        
BOUND_SF = [0.7, 1.65];
BOUND_RHO = [0, 8.5];
% the reward is learned reward -                 
with_dynamics = false;
gradient_descent = true;
proximal = false;

% Allocate training samples.
N = length(old_examples);
T = test_params.training_sample_lengths;
test_samples = old_tests;

if ~with_dynamics && proximal
    example_samples = old_examples;
    if test_params.example_recompute_optimal || test_params.test_optimal
        discrete_mdp = discretesolve(T,mdp_data,mdp,reward,test_params);
    else
        discrete_mdp = [];
    end
end

if gradient_descent
    % re-declare some variable for learning with dynamics
    mdp_data.mdp = 'obstacle_f_';
    mdp = 'obstacle_f_';
%     if mdp_data.num_obs == 2
%         mdp_data.mdp = 'obstacle_ftwo_';
%         mdp = 'obstacle_ftwo_';
%     end
end

if proximal
    % change the the objectwork dynamics..
    mdp_data.mdp = 'obstacle_'; 
    mdp = 'obstacle_';
end

% s is always here
s = [0, 4.2];
learn_reward = reward;

if (with_dynamics && proximal)||gradient_descent
    discrete_mdp = [];
    % change the dist feature to a meaningless rbf one.. 
    learn_reward.features{1} = struct('type','cartrbf','pos',[0.5, 1.3]*10,'width',5.0,'r',1.0);
    % learn_reward.features{1} = struct('type','ddist','r',1000.0,'idx',1:mdp_data.udims);
    learn_reward.theta(1) = -1;
    test_params.example_optimal = 0;
    n = length(learn_reward.theta);
    % the learned the reward is gp..can't be changed
%     learn_reward.theta(n-1:n) = [1,1];
%     learn_reward.features{n-1} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',1.0);
%     learn_reward.features{n} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',1.0);
    %learn_reward.features{1,2}.gp.inv_widths(n-1) = 0.001;
    %learn_reward.features{1,2}.gp.inv_widths(n) = 0.001;
    
    true_reward.features{1} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',1.0);
    % learn_reward.features{1} = struct('type','ddist','r',1000.0,'idx',1:mdp_data.udims);
    true_reward.theta(1) = -1;
    n = length(true_reward.theta);
    true_reward.theta(n-1:n) = [1,1];
    true_reward.features{n-1} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',1.0);
    true_reward.features{n} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',1.0);
end

%%
% Sample example trajectories.
% actually one is good.

if proximal
    i = 1;
    if verbose > 0
        fprintf(1,'Recomputing example %i of %i\n',i,N);
    end
%     [example_samples{i}.states, example_samples{i}.u, example_samples{i}.r] = ...
%         reoptimizetrajectory(example_samples{i}, mdp_data,mdp, reward, true_reward);
    [states,u,initu,r] = optimizetrajectory(s, T, mdp_data, discrete_mdp,mdp,...
        learn_reward, true_reward, test_params.example_optimal, test_params.example_restarts);
    
    % store the recomputed one...
    
    example_samples{i} = struct('s',s,'u',u,'initu',initu,'states',states,'r',r);
end
%% 
% find the most close one

if proximal
    N_grid = 25;
    rr = Inf;
    for rho = linspace(0.5, 10, N_grid)
        for sf = linspace(0.9, 2, N_grid)
            u = [ones(T,1)*rho, ones(T,1)*sf];
            states_ = feval(strcat(mdp,'control'),mdp_data,s,u);
            T1 = floor(T/5*1);
            T2 = floor(T/5*3);
            d = states_(T1:T2,:)-states(T1:T2,:);
            a = sum(sum((d).^2, 2));
            if a<rr
                rr = a;
                ubest = u;
                b_states = states_;
            end
        end
    end

    example_samples{2} = struct('s',s,'u',ubest,'initu',initu,'states',b_states,'r',r);
end

%% 
if gradient_descent
    fprintf('Computing the example based on the learned reward function\n');
    
%     reward = true_reward;
    reward = learn_reward;
    
    initu = 0;
    rbest = -Inf;
    tolGrad = 1e-3;
    maxiter = 30; %30 or 32
    alpha = 0.1/3/2; % 1e-3
    gnorm = inf; niter = 0; dx = inf; dxmin = 1e-6;
    T = floor(T*2/2);
    d_rho = alpha;
    d_sf = alpha;
    
    % Plot
    if pllot
        figure; clf; xlim([-0.5 8.5]); ylim([0.8 1.8]);     hold on
    end
    %ylim([-0.5 8.5]);
    
    if 0
        % plot the ecllipse region 
        p = calculateEllipse(5.5, 1.25, 0.56, 0.07, 7.2);
    %     p = calculateEllipse(6, 1.2, 0.32, 0.015, 7.2);
    %     p = calculateEllipse(5, 1.15, 0.32, 0.015, 7.2);
        plot(p(:,1), p(:,2), '.-')
    end
    
    % init
    % rho = rand(1)*0.7 + 0.9;
    % sf = rand(1)*8 + 0;
    % init with grid search
    rho = 4; sf = 1.3;
    if mdp_data.num_obs == 2
        GRID_BOUND = [0.8, 3.7, 0.9, 1.19];
    else
        GRID_BOUND = [2.0, 7.5, 1.0, 1.5];
%         GRID_BOUND = [2.0, 6.5, 1.0, 1.4];
    end

    STEPS = 3;
    grid = zeros(STEPS, STEPS);
    rho_ = linspace(GRID_BOUND(1), GRID_BOUND(2), STEPS);
    sf_ = linspace(GRID_BOUND(3), GRID_BOUND(4), STEPS);
    [X, Y] = meshgrid(rho_, sf_);
    pts = [X(:) Y(:)];
    R = zeros(STEPS*STEPS, 1);
    for i = 1:length(pts)
        rho = pts(i,1);
        sf = pts(i,2);
        u = [ones(T,1)*rho; ones(T,1)*sf];
        R(i,1) = re_with_bound(u, s, mdp_data, mdp, reward, rho, sf, BOUND_SF, BOUND_RHO);
    end
    C = reshape(R, size(X,1), size(X,2));
    grid = C;

    % the init rho and sf with the lowest value
    [~, I] = min(grid(:));
    [I_row, I_col] = ind2sub(size(grid), I);
    rho = rho_(I_col);
    sf = sf_(I_row);
    rho1 = rho;
    sf1 = sf;
    
    % switching
    switching = true;
%     
%     draw_heat(T, s, mdp_data, mdp, reward, BOUND_SF, BOUND_RHO)
    
    color_list = linspace(0, 1, maxiter+1);

    if mdp_data.num_obs ~= 3 % TODO
        while and(gnorm >= tolGrad, and(niter <= maxiter, dx >= dxmin))
            u = [ones(T,1)*rho; ones(T,1)*sf];

    %         r = -trajectoryreward(u, s, mdp_data, mdp, reward);
            r = re_with_bound(u, s, mdp_data, mdp, reward, rho, sf, BOUND_SF, BOUND_RHO);

            % finite difference to calculate the grad
            u_rho = [ones(T,1)*(rho+d_rho); ones(T,1)*sf];
    %         r_rho = -trajectoryreward(u_rho, s, mdp_data, mdp, reward);
            r_rho = re_with_bound(u_rho, s, mdp_data, mdp, reward, rho+d_rho, sf, BOUND_SF, BOUND_RHO);
            g_rho = (r_rho - r)/d_rho;

            u_sf = [ones(T,1)*rho; ones(T,1)*(sf+d_sf)];
    %         r_sf = -trajectoryreward(u_sf, s, mdp_data, mdp, reward);
            r_sf = re_with_bound(u_sf, s, mdp_data, mdp, reward, rho, sf+d_sf, BOUND_SF, BOUND_RHO);
            g_sf = (r_sf - r)/d_sf;

            % amplify the gradient in the rho direction. 
            g_rho = g_rho * 10;
            
            % truncate the gradient 
            truncate = 20e-1; % alpha multiplied afterwards
            if abs(g_rho) > truncate*10, g_rho = truncate*10*sign(g_rho); end
            if abs(g_sf) > truncate, g_sf = truncate*sign(g_sf); end
           
            % coordinate descent
    %         if abs(g_rho) > abs(g_sf)
            if switching
                rho_new = rho - alpha*g_rho; % go to the inverse gradient direction
                sf_new = sf;
                switching = ~switching;
            else
                rho_new = rho;
                sf_new = sf - alpha*g_sf;
                switching = ~switching;
            end

            if (~isfinite(rho_new)) && (~isfinite(sf_new))
                display(['Number of iterations: ' num2str(niter)])
                error('x is inf or NaN')
            end

            %%%
            if mdp_data.num_obs == 2
                % constraints
                if sf_new < 0.9
                    sf_new = 0.9 + 1e-2;
                elseif sf_new > 1.31
                    sf_new = 1.31 - 1e-2;
                end
                if rho_new < 0.0
                    rho_new = 0.0 + 1e-2;
                elseif rho_new > 4.5
                    rho_new = 4.5 + 1e-2;
                end
            else
                % constraints
                if sf_new < BOUND_SF(1)
                    sf_new = BOUND_SF(1) + 1e-2;
                elseif sf_new > BOUND_SF(2)
                    sf_new = BOUND_SF(2) - 1e-2;
                end
                if rho_new < BOUND_RHO(1)
                    rho_new = BOUND_RHO(1) + 1e-2;
                elseif rho_new > BOUND_RHO(2)
                    rho_new = BOUND_RHO(2) + 1e-2;
                end
            end
            %%%
            if pllot
                plot([rho rho_new], [sf sf_new],'color',[color_list(niter+1) 0 0],'marker','o')
                refresh
            end
            niter = niter + 1;
            dx = norm([rho_new-rho, sf-sf_new]);

            % update the sf and rho values.. 
            sf = sf_new;
            rho = rho_new;
            
            %update the criterion
%             gnorm = mean([abs(g_sf), abs(g_rho)]);
            if switching
                gnorm = abs(g_rho);
            else
                gnorm = abs(g_sf);
            end
        end
    elseif mdp_data.num_obs ~= 0 % this part is neglected ... 
        while and(gnorm >= tolGrad, and(niter <= maxiter, dx >= dxmin))
            u = [ones(T,1)*rho; ones(T,1)*sf; ones(T,1)*rho1; ones(T,1)*sf1];
            
            r = re_with_bound(u, s, mdp_data, mdp, reward, rho, sf, rho1, sf1, BOUND_SF, BOUND_RHO);

            % finite difference to calculate the grad
            u_rho = [ones(T,1)*(rho+d_rho); ones(T,1)*sf; ones(T,1)*rho1; ones(T,1)*sf1, BOUND_SF, BOUND_RHO];
            r_rho = re_with_bound(u_rho, s, mdp_data, mdp, reward, rho+d_rho, sf);
            g_rho = (r_rho - r)/d_rho;

            u_sf = [ones(T,1)*rho; ones(T,1)*(sf+d_sf); ones(T,1)*rho1; ones(T,1)*sf1];
            r_sf = re_with_bound(u_sf, s, mdp_data, mdp, reward, rho, sf+d_sf, BOUND_SF, BOUND_RHO);
            g_sf = (r_sf - r)/d_sf;

            % truncate the gradient 
            truncate = 1e1; % it is large before multiply alpha
            if g_rho > truncate, g_rho = truncate; end
            if g_sf > truncate, g_sf = truncate; end

            % coordinate descent
    %         if abs(g_rho) > abs(g_sf)
            if switching
                rho_new = rho - alpha*g_rho;
                sf_new = sf;
                switching = ~switching;
            else
                rho_new = rho;
                sf_new = sf - alpha*g_sf;
                switching = ~switching;
            end

            if (~isfinite(rho_new)) && (~isfinite(sf_new))
                display(['Number of iterations: ' num2str(niter)])
                error('x is inf or NaN')
            end

            %%%
            % constraints
            if sf_new < 0.9
                sf_new = 0.9 + 1e-2;
            elseif sf_new > 1.31
                sf_new = 1.31 - 1e-2;
            end
            if rho_new < 0.0
                rho_new = 0.0 + 1e-2;
            elseif rho_new > 4.5
                rho_new = 4.5 + 1e-2;
            end
            %%%

            % plot([rho rho_new],[sf sf_new],'ko-')
            if pllot
                plot([rho rho_new], [sf sf_new],'color',[1 0 0],'marker','o')
                refresh
            end
            niter = niter + 1;
            dx = norm([rho_new-rho, sf-sf_new]);

            % update the sf and rho values.. 
            sf = sf_new;
            rho = rho_new;
        end
    end
    
    ubest = u;
    niter = niter - 1;
    
    ubest = reshape(ubest,size(u,1)/2,2);
    states = feval(strcat(mdp,'control'), mdp_data, s, ubest);
    example_samples{1} = struct('s',s,'u',ubest,'initu',initu,'states',states,'r',rbest);
    
    
    [x, y] = meshgrid(rho_,sf_);
    if pllot
        plot(x,y,'k*')
    end
    hold off
end

end


function r = re_with_bound(u, s, mdp_data, mdp, reward, rho, sf, BOUND_SF, BOUND_RHO, rho2, sf2)
%   remove the negative here, since already negative at trajectory reward
%   function. 
    r = + trajectoryreward_part(u, s, mdp_data, mdp, reward);
    
    % a pre defined protocal to test gradient descent algorithm
    % r = trajectory_part_pre_define(u);
    
    if mdp_data.num_obs ~= 2
%         d = [rho - 0.0, -rho + 8, sf - 0.9, -sf + 1.6];
        d = [rho - BOUND_RHO(1), -rho + BOUND_RHO(2), sf - BOUND_SF(1), -sf + BOUND_SF(2)];
        for i = 1:length(d)
            if d(i)<0
                d(i) = 0;
            end
        end
        l1 = log(d(1)) + log(d(2));
        l2 = log(d(3)) + log(d(4)); 
        r = r + 1/-1 * (l1 + l2*3); % make the coeffecient small -> leads to change in the plot? check it 
    else
        if nargin > 7
            d = [rho - 0.0, -rho + 4, sf - 0.9, -sf + 1.3, ...
                 rho2 - 0.0, -rho2 + 4, sf2 - 0.9, -sf2 + 1.3];
            for i = 1:length(d)
                if d(i)<0
                    d(i) = 0;
                end
            end
            l1 = log(d(1)) + log(d(2)) + log(d(5)) + log(d(6));
            l2 = log(d(3)) + log(d(4)) + log(d(7)) + log(d(8)); 
            r = r + 1/-1000 * (l1 + l2*3); 
        end
    end
end


function [d1, d2] = count_index(index, STEPS)
    d1 = floor(index/STEPS);
    if mod(index, STEPS) ~= 0
        d1 = d1 + 1;
    end
    d2 = mod(index, STEPS);
    if d2 == 0
        d2 = STEPS;
    end
end


function draw_heat(T, s, mdp_data, mdp, reward, BOUND_SF, BOUND_RHO)
    % Visualize
    STEPS = 80; % 60 is relative ok speed
%     x = linspace(0.01, 7.99, STEPS);
%     y = linspace(0.901, 1.599, STEPS);
    x = linspace(BOUND_RHO(1) + 0.01, BOUND_RHO(2) - 0.01, STEPS);
    y = linspace(BOUND_SF(1) + 0.01, BOUND_SF(2) - 0.01, STEPS);
%     x = linspace(3, 7.5, STEPS);
%     y = linspace(1.1, 1.5, STEPS);
    [X, Y] = meshgrid(x, y);
    pts = [X(:) Y(:)];
    R = zeros(STEPS*STEPS,1);
    
    plot_index = 0;
    
    for i = 1:length(pts)
        rho = pts(i,1);
        sf = pts(i,2);
        u = [ones(T,1)*rho; ones(T,1)*sf];
        R(i,1) = re_with_bound(u, s, mdp_data, mdp, reward, rho, sf, BOUND_SF, BOUND_RHO);
        plot_index = plot_index+1;
        if plot_index>100
            disp(i)
            plot_index = 0;
        end
    end
    
    C = reshape(R, size(X,1), size(X,2));
    
    C = C - min(min(C));
    if max(max(C))~=0
        C = C/max(max(C));
        C = C*64;
        image(x, y, C);
    end
    
    % surf(X,Y,C)
    
end

