% Visualize MDP state space with given IRL test solution.
function visualize(test_result, example_human, ss_params, render)

if nargin < 4
    render = 0;
end
    
% Create figure.
if render
	w = 1200;
	h = 600;
	figure('Position',[20 200 w h]);
else
%     w = 1800;
    w = 900;
    h = 900;
    figure('Position',[20 50 w h]);
end
hold on;
grid on;
cla;

% Draw reward for ground truth.
% subplot(1,2,1);
% if ~render,
%     set(gca,'position',[0 0 0.5 1.0]);
% end;
% feval(strcat(test_result.mdp,'draw'),test_result.reward,...
%     test_result.example_samples,test_result.test_samples,test_result.mdp_params,test_result.mdp_data);

% Draw reward for IRL result.
% subplot(1,2,2);
if ~render
    set(gca,'position',[0.0 0.0 1.0 1.0]);
end
% feval(strcat(test_result.mdp,'draw'),test_result.irl_result.reward,...
%     test_result.irl_result.example_samples,test_result.irl_result.test_samples,test_result.mdp_params,test_result.mdp_data);

% ===============
% plot the original demonstrations (before cutting) % to comment for
% plotting the real demonstrations 
for i = 1:length(example_human)
    test_result.example_samples{i}.states = example_human{i};
end
% ===============

feval(strcat(test_result.mdp,'draw'),test_result.irl_result.reward,...
    test_result.irl_result.example_samples, test_result.example_samples, test_result.mdp_params,...
    test_result.mdp_data, ss_params);

% test_result.irl_result.example_samples



% Print metrics.
% metric_names = metricnames();
% fprintf(1,'\nTest results:\n');
% for i=1:length(metric_names),
%     fprintf(1,'%s: %f\n',metric_names{i},test_result.test_metrics(i));
% end;

% Turn hold off.
hold off;
