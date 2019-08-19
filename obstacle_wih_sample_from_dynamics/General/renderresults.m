% Render out test results.
function renderresults(dir_name,graph_only,new_test_name,metrics)

SAVED_TRANSFERS = 1;

% Load the results.
filename = '/result.mat';

% Load.
load([dir_name filename]);

if nargin > 2,
    test_name = new_test_name;
end;
if nargin < 4,
    metrics = 1:length(test_metric_names);
end;
test_name = regexprep(test_name, '_', ' ');

% Store text dump of the results.
fid = fopen([dir_name '/summary.txt'],'w');
printstats(fid,test_params,test_metric_names,...
    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,restarts,...
    series_result,transfer_result);
fclose(fid);

% Override metric names.
[test_metric_names,test_metric_units] = metricnames;

% Create options.
options.algorithms = {'ame','gpirl','maxent','optv'};
options.size = 1;
options.legend = 1;
options.scale_key = metricnames;
options.scales = 4*ones(3,length(options.scale_key));
options.scales(2,:) = zeros(1,length(options.scale_key));
options.scales(3,:) = (1/1000)*ones(1,length(options.scale_key));
options.normalizescale = zeros(1,length(options.scale_key));
%options.scales(1,1) = 5;
%options.scales(2,1) = 1;
%options.normalizescale(1) = 1;
options.scales(1,1) = 2000;
options.scales(2,1) = 0;
options.scales(3,1) = 1;

% Graph each metric for each model.
for s=metrics,
    % Assemble matrix of values for this model and metric.
    values = zeros(length(algorithms),length(mdp_params),restarts);
    for a=1:length(algorithms),
        for n=1:length(mdp_params),
            for r=1:restarts,
                values(a,n,r) = series_result{n,a,r}.test_metrics(s);
            end;
        end;
    end;
    % Graph the result.
    graphresult(test_name,test_metric_names{s},test_metric_units{s},mdp_cat_name,...
        mdp_param_names,algorithms,names,colors,order,values,options);
    % Save the graph.
    set(gcf, 'PaperPositionMode', 'auto');
    plot2svg([dir_name '/graphs/' regexprep(test_metric_names{s},' ','_') '.svg'],1);
    pause(1.0); % This is necessary in order for the save to succeed.
    % Close window.
    close all;
    if ~isempty(transfer_result),
        % Assemble matrix of transfer values.
        tvalues = zeros(length(algorithms),length(mdp_params),restarts,size(transfer_result,4));
        for a=1:length(algorithms),
            for n=1:length(mdp_params),
                for r=1:restarts,
                    for t=1:size(transfer_result,4),
                        tvalues(a,n,r,t) = transfer_result{n,a,r,t}.test_metrics(s);
                    end;
                end;
            end;
        end;
        % Graph the result.
        graphresult(test_name,test_metric_names{s},test_metric_units{s},mdp_cat_name,...
            mdp_param_names,algorithms,names,colors,order,tvalues,options);
        % Save the graph.
        set(gcf, 'PaperPositionMode', 'auto');
        plot2svg([dir_name '/graphs/' 'xfer_' regexprep(test_metric_names{s},' ','_') '.svg'],1);
        pause(1.0); % This is necessary in order for the save to succeed.
        % Close window.
        close all;
    end;
end;

if ~graph_only,
    % Store image of each policy, with results printed on the figure.
    for n=1:length(mdp_params),
        for a=1:length(algorithms),
            for r=1:restarts,
                % Plot image of policy.
                visualize(series_result{n,a,r},1);
                % Save the image.
                set(gcf, 'PaperPositionMode', 'auto');
                plot2svg([dir_name '/imgs/' algorithms{a} '.' num2str(a) '_' num2str(r) '_' mdp_cat_name mdp_param_names{n} '.svg'],1);
                pause(1.0); % This is necessary in order for the save to succeed.
                % Close window.
                close all;

                % Store images of transferred rewards.
                if ~isempty(transfer_result),
                    for t=1:min(SAVED_TRANSFERS,size(transfer_result,4)),
                        % Plot image of policy.
                        visualize(transfer_result{n,a,r,t},1);
                        % Save the image.
                        set(gcf, 'PaperPositionMode', 'auto');
                        plot2svg([dir_name '/xfer_imgs/' algorithms{a} '.' num2str(a) '_xfer_' num2str(r) 'to' num2str(t) '_' mdp_cat_name mdp_param_names{n} '.svg'],1);
                        pause(1.0); % This is necessary in order for the save to succeed.
                        % Close window.
                        close all;
                    end;
                end;
            end;
        end;
    end;
end;
