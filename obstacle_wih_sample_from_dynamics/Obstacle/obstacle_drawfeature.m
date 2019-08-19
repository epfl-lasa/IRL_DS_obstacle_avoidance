% Draw an objectworld feature -- either sum or RBF.
function obstacle_drawfeature(reward,min_r,max_r,wt,color1,color2)

if strcmp(reward.type,'sum')
    % Compute the maximum and minimum values in this sum.
    max_r = -Inf;
    min_r = Inf;
    for i=1:length(reward.features)
        % Exclude distance features from this computation.
        if ~strcmp(reward.features{i}.type,'dist')
            if reward.theta(i) > max_r
                max_r = reward.theta(i);
            end
            if reward.theta(i) < min_r
                min_r = reward.theta(i);
            end
        end
    end
    
    % Now draw each subfeature.
    color1 = [1 1 1];
    color2 = [1 1 1];
    rng_r = max_r - min_r;
    if abs(rng_r) < 1.0e-24
        c = 1.0;
    else
        c = (wt-min_r)/rng_r;
    end
    if isfield(reward,'color1')
        color1 = reward.color1 * c;
    end
    if isfield(reward,'color2')
        color2 = reward.color2 * c;
    end
    for i=1:length(reward.features)
        obstacle_drawfeature(reward.features{i},min_r,max_r,reward.theta(i),color1,color2);
    end
elseif strcmp(reward.type,'rbf')
    rng_r = max_r - min_r;
    if abs(rng_r) < 1.0e-24
        c = 1.0;
    else
        c = (wt-min_r)/rng_r;
    end
    w = sqrt(1.0/reward.width);
    strt = [reward.pos(:,1) reward.pos(:,end)] - w;
    extn = w*2.0;
    color1 = color1 * c;
    color2 = color2 * c;
    EPS = 1.0e-1;
    rectangle('Position',[strt(1) strt(2) extn extn],'Curvature',[1 1],'EdgeColor',color1,'linewidth',2);
    rectangle('Position',[strt(1)+EPS strt(2)+EPS extn-EPS*2 extn-EPS*2],'Curvature',[1 1],'EdgeColor',color2,'linewidth',2);
else
    % Nothing to draw for now.
end
