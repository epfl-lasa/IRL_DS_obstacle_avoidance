AB = states_collect{1};
AB3 = AB(1:5);
node(AB3, 'AB')
%%
ABobj = states_collect{2};
AB3obj = ABobj(1:5);
% node(AB3obj, 'ABobj')
node(AB3obj, 'ABobj', [0.78, 0.93, 1, 1, 0.84]')
% node(AB3obj, 'ABobj', [0.75, 0.28, 0.99, 0.8, 1]') % 25 2-2
% node(AB3obj, 'ABobj', [0.69, 0.04, 0.54, 0.19, 0.56]') % 2-2
%%
CD = states_collect{3};
CD3 = CD(1:5);
node(CD3, 'CD')
% node(CD3, 'CD', [0.47, 0.91, 0.81, 0.35, 0.17]'); % 26 2-2 final select
%%
CDo = states_collect{4};
CD3o = CDo(1:3);
node(CD3o, 'CDobj'); % 2-2
%%
AC = states_collect{5}(1:5);
node(AC, 'AC', [0.11, 0.87, 0.63, 1, 1]'); % 26 1-3 final select
% node(AC, 'AC');
%%
ACo = states_collect{6}(1:3);
node(ACo, 'ACobj');
%%
BD = states_collect{7}(1:3);
node(BD, 'BD')
%%
BDo = states_collect{8}(1);
% node(BDo, 'BD')
BDo = states_collect{8}(1:2);
% node(BDo, 'BD')
BDo = states_collect{8}(1:3);
node(BDo, 'BD')
%%


%%
load('/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/temp/July25/2_2/data_76.mat')
w = [0.78, 0.93, 1, 1, 0.84];
for n = 1:5
    ABobj = states_collect{2}(1:n);
    node(ABobj, 'ABobj', w(1:n)')
end

load('/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/temp/July26/2-2/data_54.mat')
w = [0.47, 0.91, 0.81, 0.35, 0.17];
for n = 1:5
    CD = states_collect{3}(1:n);
    node(CD, 'CD', w(1:n)')
end


load('/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/temp/July26/1-3/data_61.mat')
w = [0.11, 0.87, 0.63, 1, 1];
for n = 1:5
    AC = states_collect{5}(1:n);
    node(AC, 'AC', w(1:n)')
end

