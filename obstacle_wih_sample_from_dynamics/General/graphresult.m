% Graph results for a single model and metric.
function graphresult(test_name,test_metric_name,test_metric_unit,step_name,step_names,...
            algorithms,names,colors,order,values,options)

if nargin < 10,
    options = [];
end;

%w = 256;h = 320;
%w = 512; h = 640;
w = 384; h = 256;
algs = order;
showleg = 1;
maxy = [];
miny = 0;
% Parse options structure.
if ~isempty(options),
    % Adjust width and height.
    w = w*options.size;
    h = h*options.size;
    %{
    % Choose which algorithms to show.
    algs = [];
    for a=1:length(algorithms),
        if a > 1 && strcmp(algorithms{a},algorithms{a-1}),
            cnt = cnt+1;
        else
            cnt = 1;
        end;
        for i=1:length(options.algorithms),
            algnum = strcat(algorithms{a},num2str(cnt));
            if strcmp(algorithms{a},options.algorithms{i}) || ...
               strcmp(algnum,options.algorithms{i}),
                algs = [algs a];
                break;
            end;
        end;
    end;
    %}
    % Choose if we want to show the legend.
    showleg = options.legend;
    % Choose maximum value on y axis.
    for i=1:length(options.scale_key),
        if strcmp(options.scale_key{i},test_metric_name),
            maxy = options.scales(1,i);
            if size(options.scales,1) >= 2,
                miny = options.scales(2,i);
            end;
            % Rescale.
            if size(options.scales,1) >= 3,
                values = values*options.scales(3,i);
            end;
            if options.normalizescale(1,i),
                % Normalize the scales on all values.
                for a=1:size(values,1),
                    normfac = mean(values(a,1,:),3);
                    values(a,:,:) = values(a,:,:)/normfac;
                end;
            end;
        end;
    end;
end;

% Replace NaNs with max.
values(isnan(values)) = maxy*1.1;

% Compute means and standard errors.
if length(size(values)) == 4,
    means = mean(mean(values,4),3);
    errs = sqrt((1/(size(values,3)*size(values,4)))*...
        sum(sum(bsxfun(@minus,values,means).^2,4),3))/...
        sqrt(size(values,3)*size(values,4));
else
    means = mean(values,3);
    errs = std(values,[],3)/sqrt(size(values,3));
end;

% Clamp means.
fixedmeans = means;
fixedmeans(means > 1.0e10) = 0.0;
means(means > 1.0e10) = max(max(fixedmeans));
fixederrs = errs;
fixederrs(errs > 1.0e10) = 0.0;
errs(errs > 1.0e10) = max(max(fixederrs));

% Check which algorithms are missing.
missing = all(means > maxy,2);

% Create figure.
figure('Position',[20 200 w h]);

% Write title and labels.
clf;
axes('position',[.14 .14 .82 .75]);
set(gca,'FontSize',11);
%title(lower([test_name ' ' test_metric_name]));
title(sprintf(test_name,test_metric_name));
set(get(gca,'title'),'units','normalized');
set(get(gca,'title'),'position',[0.485 1.02]);
set(get(gca,'title'),'FontSize',20);
xlabel(lower(step_name));
ylabel(lower(test_metric_unit));
xly = -0.08;
if maxy > 1000,
    ylx = -0.12;
else
    ylx = -0.06;
end;
set(get(gca,'xlabel'),'units','normalized');
set(get(gca,'xlabel'),'position',[0.5 xly]);
set(get(gca,'ylabel'),'units','normalized');
set(get(gca,'ylabel'),'position',[ylx 0.5]);
set(get(gca,'xlabel'),'FontSize',18);
set(get(gca,'ylabel'),'FontSize',18);

% Set X-axis labels.
grid on;
set(gca,'xcolor',[0.8,0.8,0.8],'ycolor',[0.8,0.8,0.8]);
set(gca,'XTick',1:length(step_names));
set(gca,'XTickLabel',step_names);

% Clamp the means.
means = max(means,miny);

% Set axes size.
if isempty(maxy),
    maxy = max(max(means+errs));
end;
%miny = min(miny,min(min(means)));
axis([1  length(step_names)  miny  maxy]);

% Plot error area.
for a=algs,
    % Interpolate means and variances.
    %xi = 1:1:length(step_names);
    xi = 1:0.1:length(step_names);
    tavg = interp1(1:length(step_names),means(a,:),xi,'pchip');
    tvar = interp1(1:length(step_names),errs(a,:),xi,'pchip');
    
    % Compute top and bottom edges.
    bottom = tavg-tvar;
    top = tavg+tvar;
    xs = xi;
    
    % Draw the patch.
    patch([xs,fliplr(xs)],[bottom,fliplr(top)],...
            colors{a}*0.6+ones(1,3)*0.4,'EdgeColor','none','FaceAlpha',0.33);
end;

% Plot result.
hold on;
leg = zeros(length(algs),1);
i = 1;
for a=algs,
    % Interpolate result.
    %xi = 1:1:length(step_names);
    xi = 1:0.1:length(step_names);
    tavg = interp1(1:length(step_names),means(a,:),xi,'pchip');
    tavgm = interp1(1:length(step_names),means(a,:),1:length(step_names),'pchip');
    xs = xi;
    
    % Choose line width.
    alg = algorithms{a};
    if strcmp(alg,'ame'),
        lw = 2;
    elseif strcmp(alg,'gpirl'),
        lw = 2;
    else
        lw = 1;
    end;
    
    % Plot.
    if missing(a),
        leg(i,1) = plot(xs,tavg,'s','Color',colors{a},'LineWidth',lw,'MarkerSize',4,'MarkerFaceColor','none','MarkerEdgeColor','none');
    else
        leg(i,1) = plot(xs,tavg,'Color',colors{a},'LineWidth',lw,'MarkerSize',4);
        plot(1:length(step_names),tavgm,'s','Color',colors{a},'MarkerFaceColor',colors{a},'LineWidth',lw,'MarkerSize',4);
    end;
    i = i+1;
end;

% Create the legend.
if showleg,
    % Create legend.
    h = legend(leg,names(algs));
    set(h,'FontSize',14);
    set(h, 'Location', 'NorthEast');
    set(h, 'Box', 'off');
    set(h, 'Color', 'none');
    % Mark missing plots as such.
    c = get(h,'children');
    % Figure out the position of each missing entry.
    for a=1:length(names),
        if missing(a),
            linedata = [];
            textdata = [];
            for i=1:length(c),
                tag = get(c(i),'tag');
                type = get(c(i),'type');
                if strcmp(names{a},tag),
                    % This is a line, set x position.
                    linedata = get(c(i),'xdata');
                end;
                if strcmp(type,'text') && strcmp(get(c(i),'String'),names{a}),
                    % This is the text, set y position.
                    textdata = get(c(i),'position');
                end;
            end;
            t = text(linedata(1)-(linedata(2)-linedata(1))*0.63,textdata(2),'off scale','units','data','color',colors{a});
            set(t,'FontSize',14);
            set(t,'parent',get(c(i),'parent'));
        end;
    end;
    %{
    for i=1:length(c),
        % Check if this child has a tag.
        tag = get(c(i),'tag');
        for a=1:length(names),
            if strcmp(names{a},tag) && missing(a),
                % This is a missing one.
                y = get(c(i),'ydata');
                x = get(c(i),'xdata');
                t = text(x(1),y(1),'off scale','units','data','color',colors{a});
                set(t,'FontSize',14);
                set(t,'parent',get(c(i),'parent'));
            end;
        end;
    end;
    %}
end;

% Create black axes without grid.
c_axes = copyobj(gca,gcf);
set(c_axes, 'color', 'none', 'xcolor', 'k', 'xgrid', 'off', 'ycolor','k', 'ygrid','off');
title(c_axes,'');
set(get(c_axes,'xlabel'),'units','normalized');
set(get(c_axes,'xlabel'),'position',[0.5 xly]);
set(get(c_axes,'ylabel'),'units','normalized');
set(get(c_axes,'ylabel'),'position',[ylx 0.5]);

% Clean up.
hold off;
