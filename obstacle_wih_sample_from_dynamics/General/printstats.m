% Print the results for a series test.
function printstats(fid,test_params,test_metric_names,...
    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,restarts,...
    series_result,transfer_result)

% Gather statistics.
scores = zeros(length(test_metric_names),...
               length(mdp_params),...
               length(algorithms),...
               restarts);
if ~isempty(transfer_result),
    tscores = zeros(length(test_metric_names),...
                   length(mdp_params),...
                   length(algorithms),...
                   restarts,size(transfer_result,4));
end;
for n=1:length(mdp_params),
    for a=1:length(algorithms),
        for r=1:restarts,
            for s=1:length(test_metric_names),
                scores(s,n,a,r) = series_result{n,a,r}.test_metrics(s);
                if ~isempty(transfer_result),
                    for t=1:size(transfer_result,4),
                        tscores(s,n,a,r,t) = transfer_result{n,a,r,t}.test_metrics(s);
                    end;
                end;
            end;
        end;
    end;
end;

% Report results.
fprintf(fid,'IRL RESULTS:\n');
for n=1:length(mdp_params),
    fprintf(fid,'\n%s %s results:\n',mdp_cat_name,mdp_param_names{n});
    for s=1:length(test_metric_names),
        for a=1:length(algorithms),
            fprintf(fid,'%s: %s \t %f\n',test_metric_names{s},names{a},mean(scores(s,n,a,:),4));
        end;
    end;
end;
if ~isempty(transfer_result),
    fprintf(fid,'\nTRANSFER RESULTS:\n');
    for n=1:length(mdp_params),
        fprintf(fid,'\n%s %s results:\n',mdp_cat_name,mdp_param_names{n});
        for s=1:length(test_metric_names),
            for a=1:length(algorithms),
                fprintf(fid,'%s: %s \t %f\n',test_metric_names{s},names{a},mean(mean(tscores(s,n,a,:,:),5),4));
            end;
        end;
    end;
end;
