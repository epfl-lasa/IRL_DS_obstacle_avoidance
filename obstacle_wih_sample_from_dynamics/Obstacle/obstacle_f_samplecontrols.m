% Sample some random controls in the Objectworld.
function u = obstacle_f_samplecontrols(n,mdp_data)

% u = randn(n,mdp_data.udims)*5.0;

% u = rand(n,2)*[7, 0; 0, 0.6] + ones(n,2)*[1.0,0;0,1.0];

% u = rand(n,2)*[3, 0; 0, 0.1] + ones(n,2)*[3.0, 0; 0, 1.4];
u = ones(n,2)*[3.5, 0; 0, 1.45];