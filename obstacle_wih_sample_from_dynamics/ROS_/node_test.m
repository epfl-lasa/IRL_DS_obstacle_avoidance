% for building filtered input

function node_test(sIn)

for j = 1:length(sIn)
    states = sIn{j};
    T = length(states);
    
    % for i = 1:T
    %     states(i,3) = scandata.Poses(i).Position.X;
    %     states(i,1) = scandata.Poses(i).Position.Y;
    %     states(i,2) = scandata.Poses(i).Position.Z;
    % end
    % plot(states(:,3), states(:,1), states(:,2))

    % resacle the states
    rangex = [min(states(:,1)) max(states(:,1))];
    rangey = [min(states(:,2)) max(states(:,2))];
    a = [abs(rangex(1)), 0];
    
    states_r = (states+repmat(a,length(states),1))./(rangex(2)-rangex(1)).*10;
    
    if (states_r(T,1)<states_r(1,1))
        states_r(:,1) = -states_r(:,1) + 10;
        % states_r = flipud(states_r);
    end
    
    figure;plot(states_r(:,1),states_r(:,2))
    
    % apply soft ReLU
    softrelu = 0;
    if softrelu
        s = (states_r(:,2)-4.0)*10;
        ss = log(1+exp(s));
        states_r(:,2) = ss/10 + 4.2;
    end
    
        % put the first point at 0,4.2
    dd = 4.2 - states_r(1,2);
    states_r(:,2) = states_r(:,2) + dd;
          
    % sub sampleing
    index = linspace(50, T-50, 50);
    index = floor(index);
    states_tbl = states_r(index, :);
    
    % split the trajectory to 2 parts .. 
%     lll = length(states_tbl);
%     threshold_h = 0.7;
%     states_tbl = states_tbl(states_tbl(:,1) < max(states_tbl(:,1))*threshold_h, :);
        
    % use the trajectory for learning
    
    states_{j} = states_tbl;
%    [rho, sf] = obstacle_test(2,1,1,1,'sim', states_);

end

% the break point here
a =1