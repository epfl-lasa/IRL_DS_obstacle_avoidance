% Draw single objectworld with specified reward function.
function obstacle_draw(reward,example_samples,test_samples,mdp_params,mdp_data, ss_params, paper_quality)

if nargin < 7
    paper_quality = 0;
end

% Initialize window.'states',states,'states',states,
if paper_quality
    VMARGIN = 0.4;
    HMARGIN = 0.4;
else
    VMARGIN = 1.0;
    HMARGIN = 1.0;
end

% axis([-HMARGIN  mdp_data.bounds(1)+HMARGIN  -VMARGIN  mdp_data.bounds(2)+VMARGIN]);
axis([-HMARGIN  mdp_data.bounds(1)+HMARGIN  3.2  8.5]);
if paper_quality
    % Expand axes.
    set(gca,'position',[0 0 1 1]);
    set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
else
%     set(gca,'xtick',0:mdp_data.bounds(1));
%     set(gca,'ytick',0:mdp_data.bounds(2));
end
daspect([1 1 1]);
hold on;

% Draw the reward function.
% Create regular samples.
STEP_SIZE = 0.2;
x = -HMARGIN:STEP_SIZE:(mdp_data.bounds(1)+HMARGIN);
y = -VMARGIN:STEP_SIZE:(mdp_data.bounds(2)+VMARGIN);
[X,Y] = meshgrid(x,y);
pts = [X(:) Y(:)];
R = feval(strcat(reward.type,'evalreward'),reward,mdp_data,[],zeros(size(pts,1),mdp_data.udims),pts * mdp_data.sensor_basis,[],[],[],[]);

% Convert back into image.
C = reshape(R,size(X,1),size(X,2));

% Interpolate.
C = interp2(C,4);

% Visualize.
C = C - min(min(C));
if max(max(C))~=0
	C = C/max(max(C));
    C = C*64;
	fig = image(x, y, C);
end

ylim([3.2 8.5])

% Draw feature positions. % seems like only rbf rectangular
obstacle_drawfeature(reward,0.0,0.0,1.0,[],[]);

if strcmp(reward.type,'sum')
    max_r = -Inf;
    min_r = Inf;
    for i=1:length(reward.features)
        if strcmp(reward.features{i}.type,'rbf')
            if reward.theta(i) > max_r
                max_r = reward.theta(i);
            end
            if reward.theta(i) < min_r
                min_r = reward.theta(i);
            end
        end
    end
    rng_r = max_r-min_r;
    for i=1:length(reward.features)
        if strcmp(reward.features{i}.type,'rbf')
            c = (reward.theta(i)-min_r)/rng_r;
            w = sqrt(1.0/reward.features{i}.width);
            strt = [reward.features{i}.pos(:,1) reward.features{i}.pos(:,end)] - w;
            extn = w*2.0;'states',states,
            rectangle('Position',[strt(1) strt(2) extn extn],'Curvature',[1 1],'EdgeColor',[c c c]);
        end
    end
end

% Draw the paths.
if ~isempty(example_samples)
     % draw trajectories ... vvv
    if ~paper_quality
        for i=1:length(test_samples)
            % Collect all points in this trajectory.
            % pts = objectworldcontrol(mdp_data,test_samples{i}.s,test_samples{i}.u);
            %pts = [test_samples{i}.s; pts];
            pts = test_samples{i}.states;

            % comment the next line to plot successfully when we segement the
            % path into two parts
    %         pts = [test_samples{i}.s; pts]; % add the starting point for drawing
            col = [0.5 0.5 0.7];
            color_list = linspace(0, 1, length(test_samples)+1);
            % Plot the points.
            if 0
                plot(pts(:,1),pts(:,end),'-','color',[color_list(i),0.5,0.7],'marker','.','markersize',14,'linewidth',1.5);
            else
    %             color1 = [0.4660 0.8 0.1880];
                color1 = [0, 0, 0];
                color2 = [1, 0.0, 0.0];
                sizem = 5;
                switch i
                    case 1
                        plot(pts(:,1),pts(:,end),'-','color',color1,'marker','o','markersize',sizem+1,'linewidth',1.5,...
                            'MarkerFaceColor','m'); 
                    case 2
                        plot(pts(:,1),pts(:,end),'-','color',color1,'marker','^','markersize',sizem,'linewidth',1.5,...
                            'MarkerFaceColor','m'); 
                    case 3
                        plot(pts(:,1),pts(:,end),'-','color',color1,'marker','v','markersize',sizem,'linewidth',1.5,...
                            'MarkerFaceColor','m');    
                    case 4
                        if  ss_params.num_train_demo == 5
                            plot(pts(:,1),pts(:,end),'-','color',color1,'marker','d','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color1,...
                                'MarkerFaceColor','m');
%                         elseif ss_params.num_train_demo == 3
                        else
                            plot(pts(:,1),pts(:,end),':','color',color2,'marker','d','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color2,...
                            'MarkerFaceColor','y');
                        end
                    case 5
                        if ss_params.num_train_demo == 5
                            plot(pts(:,1),pts(:,end),'-','color',color1,'marker','s','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color1,...
                                'MarkerFaceColor','m');
%                         elseif ss_params.num_train_demo == 3
                        else
                            plot(pts(:,1),pts(:,end),':','color',color2,'marker','s','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color2,...
                                'MarkerFaceColor','y');
                        end
                    case 6
                        plot(pts(:,1),pts(:,end),'-','color',color2,'marker','o','markersize',sizem+1,'linewidth',1.5,...
                            'MarkerFaceColor','y'); 
                    case 7
                        plot(pts(:,1),pts(:,end),'-','color',color2,'marker','^','markersize',sizem,'linewidth',1.5,...a
                            'MarkerFaceColor','y'); 
                    case 8
                        plot(pts(:,1),pts(:,end),'-','color',color2,'marker','v','markersize',sizem,'linewidth',1.5,...
                            'MarkerFaceColor','y');    
                    case 9
                        plot(pts(:,1),pts(:,end),'-','color',color2,'marker','d','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color2,...
                            'MarkerFaceColor','y');
                    case 10
                        plot(pts(:,1),pts(:,end),'-','color',color2,'marker','s','markersize',sizem,'linewidth',1.5,'MarkerEdgeColor',color2,...
                            'MarkerFaceColor','y');  
                end
            end
            % Plot starting point.
            %plot(pts(1,1),pts(1,end),'color',col,'marker','o','markersize',5,'linewidth',2);
            % Plot ending point.
            fig_end = plot(pts(end,1),pts(end,end),'color',col,'marker','x','markersize',10,'linewidth',2);
            set(get(get(fig_end,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        end
    end
    
    for i=1:length(example_samples)
        % Collect all points in this trajectory.
        % pts = obstacle_control(mdp_data,example_samples{i}.s,example_samples{i}.u);
        % changed to read states directly
        pts = example_samples{i}.states;
%        pts = example_samples{i}.states_draw;
        pts = [example_samples{i}.s; pts];
        T = size(pts,1);
        col = ones(1,3)*0.0;
        % Plot the points.
        if paper_quality
            width_factor = 2;
        else
            width_factor = 1;
        end
        % this three lines are commented for ploting
        plot(pts(:,1),pts(:,end),'-','color',col,'marker','.','markersize',14*width_factor,'linewidth',1.5);
        
        % Plot starting point.
        %plot(pts(1,1),pts(1,end),'color',col,'marker','o','markersize',5,'linewidth',2);
        % Plot ending point.
        
        fig = plot(pts(end,1),pts(end,end),'color',col,'marker','x','markersize',10*width_factor,'linewidth',2);
        set(get(get(fig,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end

N = ss_params.weight_legend;

legendCell = cellstr(num2str(N, 'Weight=%-g'));
legendCell = [legendCell; cellstr('IRL generated')];
legend(legendCell,'FontSize',22)

x0 = 0;
y0 = 0;
width = 1080;
height = 530;
set(gcf,'position',[x0,y0,width,height])
set(gca,'xtick',[])

str_indicator = ss_params.indicator;
% if (contains(ss_params.indicator, 'AB')) 
%     if (~contains(str_indicator, 'obj'))
%         title(gca, 'AB without object')
%     else
%         title(gca, 'AB with object')
%     end
%     movegui('northeast')
% elseif (contains(ss_params.indicator, 'BD')) 
%     
%     if (~contains(str_indicator, 'obj'))
%         title(gca, 'BD without object')
%     else
%         title(gca, 'BD with object')
%     end
%     movegui('northwest')
% elseif (contains(ss_params.indicator, 'CD')) 
%     
%     if (~contains(str_indicator, 'obj'))
%         title(gca, 'CD without object')
%     else
%         title(gca, 'CD with object')
%     end
%     movegui('southeast')
% elseif (contains(ss_params.indicator, 'AC')) 
%     
%     if (~contains(str_indicator, 'obj'))
%         title(gca, 'AC without object')
%     else
%         title(gca, 'AC with object')
%     end
%     movegui('southwest')
% end

drawnow;
% my_title_pre = strcat('result/example/demo_', num2str(length(test_samples)));
% my_title_pre = strcat('result/weights_test/test_far_4_close_3/demo_', num2str(length(test_samples)));
% my_title_pre = strcat('result/Tuning_obs_axis/demo_', num2str(length(test_samples)));
my_title_pree = strcat(ss_params.folderName, str_indicator);
my_title_pre = strcat(my_title_pree, num2str(length(test_samples)));
my_title = strcat(my_title_pre, '.jpg');

my_title_fig = strcat(my_title_pre, '.fig');
saveas(gcf, my_title)
savefig(my_title_fig)

% Finished.
hold off;
