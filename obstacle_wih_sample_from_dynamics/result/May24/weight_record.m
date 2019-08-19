%%
% May 24
% == test4 remove 5 limit
weights4 = [0.35, 0.75, 0.83, 0.89, 0.93, 0.36,...
    0, 0.89, 0, 0];
weights4 = 1 - weights4;

weights4 = [0.35, 0.75, 0.83, 0.89, 0.93];
weights4 = 1 - weights4;

weights2 = [0.73, 0, 0.79, 0.36, 0.57];
weights2 = 1 - weights2;

weights3 = [0.52, 0, 0, 0, 0.64];
weights3 = [0, 0, 0, 0.64, 0.84];
weights3 = 1 - weights3;

node(s_test4_f5, weights4)