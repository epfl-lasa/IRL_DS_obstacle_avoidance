% Sample some random controls in the Objectworld.
function u = obstacle_samplecontrols(n,mdp_data)

u = randn(n,mdp_data.udims)*1.0; % 5.0
