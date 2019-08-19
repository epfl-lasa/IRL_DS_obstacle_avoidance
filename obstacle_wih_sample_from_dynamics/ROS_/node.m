function node(states, weight_in, patht)

delayIntro = false; % should be false

% folderName = 'result/eight_subject/Jun_12_02/testd2/';
folderName = 'result/temp/';

if nargin<1 % if there is a input argument, then skip the ROS node creation (if false)
    % create a ros node
    if(~exist('node1','var'))
%         node1 = robotics.ros.Node('/irl_parameter_update1','128.178.145.170');
        node1 = robotics.ros.Node('/irl_parameter_update1');
    end

    % matlab function for subscribing
    if(~exist('sub','var'))
        sub = robotics.ros.Subscriber(node1, ...
            '/motion_generator_to_parameter_update', 'geometry_msgs/PoseArray');
    end

%     if(~exist('sub_weight','var'))
%         sub_weight = robotics.ros.Subscriber(node1, ...
%             '/motion_generator_to_parameter_update_weight', 'std_msgs/Float32');
%     end    
    
     % matlab function for publish
    if(~exist('pub','var'))
        pub = robotics.ros.Publisher(node1, ...
            '/parameters_tuning', 'std_msgs/Float32MultiArray');
    end

    if(~exist('msg','var'))
        msg = rosmessage('std_msgs/Float32MultiArray');
    end
    
    % one more message record the mouse, which is unneccessary for now.
%     if(~exist('sub_mouse','var'))
%         folderpath = "~/catkin_ws/src";
%         rosgenmsg(folderpath)
%        folderpath = "~//Downloads/Untitled Folder/";
%         sub_mouse = robotics.ros.Subscriber(node1, ...
%             '/mouse_message_update_to_irl', 'mouse_perturbation_robot/MouseMsgPassIRL');
%     end

    states_ = cell(1,1);
    
    save_sf_rho = zeros(2,1);
    save_time_elapsed = zeros(1,1);
end

j = 1;
% need to inverse the x 
weight_input = ones(1,1);

% while 1
    if nargin < 1
        scandata = receive(sub);
        disp('got trajctory')
        % scanweight = receive(sub_weight);
        % disp('got weight')
        if delayIntro
            scandat2 = receive(sub_mouse);
            disp('got trajctory (mouse)')
            % unpack the mouse message
            msg_mouse = zeros(size(scandat2.Xyz,1),3);
            for i = 1:size(scandat2.Xyz,1)
                msg_mouse(i,1) = scandat2.Xyz(i,1).Position.X;
                msg_mouse(i,2) = scandat2.Xyz(i,1).Position.Y;
                msg_mouse(i,3) = scandat2.Xyz(i,1).Position.Z;
            end
        end
   tic
        % unpack pose data to trajectory in 2D
        T = length(scandata.Poses);
        states = zeros(T,2);
        for i = 1:T
            states(i,1) = scandata.Poses(i).Position.Y;
            states(i,2) = scandata.Poses(i).Position.Z;
%             states(i,1) = scandata.Poses(i).Position.X;
%             states(i,2) = scandata.Poses(i).Position.Y;
%             states(i,3) = scandata.Poses(i).Position.Z;
        end
%         figure;plot(states(:,1), states(:,2))
%         figure;plot3(states(:,1), states(:,2), states(:,3))
                
%        weight_input(i,1) = scanweight.data;
        if scandata.Header.FrameId == ""
            weight_input(j,1) = 1;
            disp('No weight specified.')
        else
            weight_input(j,1) = 1 - str2double(scandata.Header.FrameId); % 1 - 
            if weight_input(j,1) == 0
                weight_input(j,1) = 0.0001;
            end
            disp(['weight recieved : ', num2str(weight_input(j,1))])
        end
        
        % save the weight
        save([folderName 'weight_input.mat'], 'weight_input')
        
        % Raise error when empty data received
        if isempty(states)
            error('Empty demonstration provided.')
        end

        % store
        save([folderName 'data_' num2str(j) '.mat'], 'states');

        % resacle the states
        rangex = [min(states(:,1)) max(states(:,1))];
        rangey = [min(states(:,2)) max(states(:,2))];
        a = [abs(rangex(1)), 0];

        % reverse x
        states_r = (states+repmat(a,length(states),1))./abs(rangex(2)-rangex(1)).*10;

        if (states_r(T,1) < states_r(1,1))
            states_r(:,1) = -states_r(:,1) + 10;
            % states_r = flipud(states_r);
        end

        % states_r(:,2) = -states_r(:,2);
%         figure;plot(states_r(:,1),states_r(:,2))
        
        % ReLU (since the robot arm end effector will overshoot to low height)
        softrelu = 0;
        if softrelu
            s = (states_r(:,2)-3.1)*10;
            ss = log(1+exp(s));
            states_r(:,2) = ss/10 + 4.2;% - 0.5;
        end
        
%         put the first point at 0,4.2
        dd = 4.2 - states_r(1,2);
        % two obs
%         dd = 5 - states_r(1,2);
        states_r(:,2) = states_r(:,2) + dd;
        
        % sub sampleing
        lll = 50; % set the length to be 50
        index = linspace(lll, T-lll, 50);
        index = floor(index);
        states_tbl = states_r(index, :);
%         states_tbl = flip(states_tbl);
        % use the trajectory for learning
        states_{j} = states_tbl;
        %if length(states_) > 5
        %   states_ = states_(2:end);
        %end
    else 
        T = length(states);
        states_ = states;
        weight_input = ones(T,1);
        if nargin > 1
            weight_input = weight_in;
        end    
    end

    SIGMOID = 1;
    if SIGMOID
        % sigmoid
        weight_input2 = weight_input*10;
%         weight_input = sigmoid(weight_input,5,2);
        weight_input2 = sigmoid(weight_input2,5,1);
    end
    
    if nargin > 2
        [rho, sf] = obstacle_test(1,1,1,1,'sim', states_, weight_input2, folderName, patht);
    else
        [rho, sf] = obstacle_test(2,2,1,1,'sim', states_, weight_input2, folderName);
    end
    
    % First parameter: 1 use ame, 2 use gpirl. [Tuning reminder]
    % Should be fixed to be 2.. ame performace is very poor

    msg.Data(1) = rho;
    msg.Data(2) = sf;

    disp('sending')
    disp(rho)
    disp(sf)
    
    save_sf_rho(:,j) = [sf; rho];
    
    if exist('pub','var')
        send(pub, msg);
    end
    
    elapsedTime = toc
    
    save_time_elapsed(1,j) = elapsedTime;
    save_ = cell(1,2);
    save_{1} = save_sf_rho;
    save_{2} = save_time_elapsed;
    save('save_.mat', 'save_');
   
    j = j +1;
    if nargin >= 1
%         pause(100000)
    end
% end

