% Save out test results.
function saveresults(test_name,test_params,test_metric_names,...
    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,colors,order,...
    restarts,series_result,transfer_result)

% Create directory.
timestamp = datestr(now);
% Replace spaces with underscores.
timestamp = regexprep(timestamp,' ','_');
% Replace : with .
timestamp = regexprep(timestamp,':','.');
dir_name = ['Results/' test_name '_' timestamp];
mkdir(dir_name);
mkdir([dir_name '/graphs']);
mkdir([dir_name '/imgs']);
if ~isempty(transfer_result),
    mkdir([dir_name '/xfer_imgs']);
end;

% Save workspace variables to be restored later.
save([dir_name '/result.mat'],'test_name','test_params','test_metric_names',...
    'mdp_params','mdp_cat_name','mdp_param_names',...
    'algorithms','names','colors','order','restarts','series_result',...
    'transfer_result');
