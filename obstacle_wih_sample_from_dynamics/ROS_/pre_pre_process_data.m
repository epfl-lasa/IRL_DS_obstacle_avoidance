path = '/home/swei/Documents/IRL_with_dynamical_system/obstacle_wih_sample_from_dynamics/result/eight_subject/';

path_A = ['Jun_10_01/'; 'Jun_10_02/'; 'Jun_12_01/'; 'Jun_12_02/'];
path_B = ['testa1/'; 'testa2/'; 'testb1/'; 'testb2/'; 'testc1/'; 'testc2/'; 'testd1/'; 'testd2/'];

% path_A = ['Jun_10_01/'; 'Jun_10_02/'];
% path_B = ['testd1/'; 'testd2/'];

path_A = ['Jun_06_01/'; 'Jun_06_02/';];
path_A = ['Jun_05_01/'; 'Jun_05_02/';];
path_B = ['testa1/'; 'testa2/'; 'testb1/'; 'testb2/'; 'testc1/'; 'testc2/'; 'testd1/'; 'testd2/'];
path_B = ['testa1/'; 'testa2/'; 'testb1/'; 'testb2/'];
% path_A = ['Jun_06_02/';];
% path_B = ['testd2/'];

Na = size(path_A);
Nb = size(path_B);

for path_a = 1:Na
    for path_b = 1:Nb
        pathh = [path, path_A(path_a,:), path_B(path_b,:)];
        pre_process_data(pathh);

    end
end