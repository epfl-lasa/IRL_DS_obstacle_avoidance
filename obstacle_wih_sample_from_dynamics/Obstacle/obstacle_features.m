% Construct the features and reward function for the objectworld domain.
function [reward,features_pt,features_dyn] = obstacle_features(mdp_params,mdp_data)

POSITIVE = 10;
NEGATIVE = -15;

if mdp_params.rbf_features == 1
    % First create the features, which for now are RBF functions centered at
    % each object.
elseif mdp_params.rbf_features == 2
    features_pt = cell(1,1);
    
    features_dyn = cell(1,1);
    
    features_dyn{1} = struct('type','dist','r',-1.0,'idx',1:mdp_data.udims);

    features_pt{1} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                            'width', 0.125, 'r', 1.0, 'E', [0.5,0;0,1]);
    features_pt{2} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)], ...
                            'width', 0.8, 'r', 1.0, 'E', [0.5,0;0,1]);
    features_pt{3} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                            'width', 1.5, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
%                            'width',1.0,'r',2.0);
%     features_pt{3} = struct('type','cartrbf','pos',[mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
%                            'width',1.0,'r',2.0);
%     features_pt{length(features_pt)+1} = struct('type','onedim','direction','x','r',1.0,'width',0.05);
    % features_pt{length(features_pt)+1} = struct('type','onedim','direction','y','r',1.0,'width',0.5);
%     features_pt{length(features_pt)+1} = struct('type','cartrbf','pos',...
%         [mdp_data.objects(length(features_pt)+1).pos(:,1), ...
%         mdp_data.objects(length(features_pt)+1).pos(:,end)],'r',1.0,'width',0.05);
% 2 one_dim and 1 cartrbf upwards

    theta = zeros(1,length(features_pt)+1);
    theta(1) = mdp_params.step_cost; % dist
    for i=1:length(mdp_data.objects)
        if mdp_data.objects(i).c1 == 1
            theta(i+1) = POSITIVE;
        elseif mdp_data.objects(i).c1 == 2
            theta(i+1) = NEGATIVE;
        else
            theta(i+1) = mdp_data.objects(i).c1;
        end
    end
else
    % For each object type, create a seperate sum feature with RBF functions
    % centered at each object type.
    features_pt = cell(1,mdp_params.c1*mdp_params.c2);
    features_dyn = cell(1,1);
    features_dyn{1} = struct('type','dist','r',-1.0);
    theta = zeros(1,length(features_pt)+length(features_dyn));
    theta(1) = mdp_params.step_cost;
    empty_features = [];

    % Create colors.
    shapeColors = lines(mdp_params.c1+mdp_params.c2);
    
    % Dump c1 and c2 into arrays.
    c1s = zeros(1,length(mdp_data.objects));
    c2s = zeros(1,length(mdp_data.objects));
    idxs = zeros(1,length(mdp_data.objects));
    for i=1:length(mdp_data.objects)
        c1s(i) = mdp_data.objects(i).c1;
        c2s(i) = mdp_data.objects(i).c2;
        idxs(i) = (c1s(i)-1)*mdp_params.c2 + c2s(i);
    end

    % Create RBFs for each pair of object IDs.
    for c1=1:mdp_params.c1
        for c2=1:mdp_params.c2
            % Compute index.
            idx = (c1-1)*mdp_params.c2 + c2;

            % Count number of matching objects.
            objs = find(idxs == idx);

            % Check for empty objects list.
            if ~isempty(objs)
                % Create rbfs.
                rbfs = cell(1,length(objs));
                for i=1:length(objs) 
                    rbfs{i} = struct('type','cartrbf','pos',[mdp_data.objects(objs(i)).pos(:,1) mdp_data.objects(objs(i)).pos(:,end)],...
                                     'width',mdp_params.feature_radius,'r',1.0);
                end

                % Choose colors.
                color1 = shapeColors(c1,:);
                color2 = shapeColors(mdp_params.c1+c2,:);
                
                % Create the sum feature.
                features_pt{idx} = struct('type','sum','theta',ones(1,length(objs)),'features',{rbfs},'color1',color1,'color2',color2);

                % Set the weight on this feature.
                if c1 == 1
                    theta(idx+length(features_dyn)) = POSITIVE;
                elseif c1 == 2
                    theta(idx+length(features_dyn)) = NEGATIVE;
                end
            else
                features_pt{idx} = [];
                empty_features = [empty_features idx];
            end
        end
    end
    
    % Remove the empty features.
    features_pt(empty_features) = [];
    theta(empty_features + length(features_dyn)) = [];
end

% Create the reward.
reward = struct('type','sum','theta',theta,'features',{[features_dyn features_pt]});


% Now create seperate features if desired.
if strcmp(mdp_params.feature_type,'simple')
    % Simple features - simply identity mappings for the states.
    features_pt = cell(1,mdp_data.dims);
    for i=1:mdp_data.dims
        features_pt{i} = struct('type','id','idx',i,'r',1.0);
    end
elseif strcmp(mdp_params.feature_type,'cartesian')
    % Simple features corresponding to Cartesian coordinates.
    features_pt = cell(1,2);
    features_pt{1} = struct('type','id','idx',1,'r',1.0);
    features_pt{2} = struct('type','id','idx',mdp_data.dims,'r',1.0);
elseif strcmp(mdp_params.feature_type,'grid')
    % RBF grid.
    GRID_STEPS = mdp_params.grid_feature_steps;
    OFFSET = mdp_params.grid_feature_start;
    STEP_SIZE = mdp_params.grid_feature_step;
    features_pt = cell(1,GRID_STEPS*GRID_STEPS);
    for x=1:GRID_STEPS
        for y=1:GRID_STEPS
            i = (y-1)*GRID_STEPS+x;
            pos = [x-1 y-1]*mdp_params.size*STEP_SIZE + [OFFSET OFFSET]*mdp_params.size;
            features_pt{i} = struct('type','cartrbf','pos',pos,...
                              'width',mdp_params.feature_radius,'r',1.0);
        end
    end
elseif strcmp(mdp_params.feature_type,'obs')&& mdp_params.fixed_pattern == 3
    % obstacle feature
    features_pt = cell(1,1);
    features_pt{1} = struct('type','cartrbf','pos',[mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                          'width',0.5,'r',2.0);
    
%     features_pt{1} = struct('type', 'cartebf', 'pos', [mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
%                             'width', 0.125, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{2} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)], ...
%                             'width', 0.8, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{3} = struct('type', 'cartebf', 'pos', [mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
%                             'width', 1.5, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{4} = struct('type', 'cartebf', 'pos', [mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
%                             'width', 0.05, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{5} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)], ...
%                             'width', 1.0, 'r', 1.0, 'E', [0.5,0;0,1]);
%     features_pt{6} = struct('type', 'cartebf', 'pos', [mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
%                             'width', 2.0, 'r', 1.0, 'E', [0.5,0;0,1]);                    
%       width = linspace(0.3, 2.0, 16);
    width = linspace(0.1, 1.5, 10); 
%     width = [1, 0.5]; 
%    axis1 = [0.5, 0.8, 1];
    axis1 = [0.6, 0.8, 1.0]; 
%     axis1 = [0.6, 1.0]; 
%     axis1 = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.5];
    for k = 1:length(axis1)
        for i = 1:length(width)
            index = (k-1)*length(width) + i;
    
        features_pt{index} = struct('type', 'cartebf', 'pos', [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                                'width', width(i), 'r', 1.0, 'E', [axis1(k), 0; 0, 1]);
        end
    end
    
%     features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
%                           'width',2.0,'r',2.0);
%     features_pt{3} = struct('type','cartrbf','pos',[mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
%                           'width',2.0,'r',2.0);
    %features_pt{length(features_pt)+1} = struct('type','onedim','direction','x','r',1.0,'width',0.05);
    % features_pt{length(features_pt)+1} = struct('type','onedim','direction','y','r',1.0,'width',0.5);
    nn = length(features_pt);
    % build some overlapping features on center
    if 0
        GRID_STEPS = 1;
        OVERLAP_STEPS = 8;
        RANGE_VAR = [0.4, 1.5]; % 0.4 to 0.8
        width = linspace(RANGE_VAR(1), RANGE_VAR(2), OVERLAP_STEPS);
        for x=1:GRID_STEPS
            for y=1:GRID_STEPS
                for k = 1:OVERLAP_STEPS
                    i = (y-1)*GRID_STEPS + x + nn;
                    i = i + k-1;
                
                    pos = features_pt{1}.pos;
                    % [x-1 y-1]*mdp_params.size*STEP_SIZE + [OFFSET OFFSET_y]*mdp_params.size;
                    
                    features_pt{i} = struct('type','cartrbf','pos',pos,...
                                            'width',width(k),'r',1.0);
                end
            end
        end
%         nn = length(features_pt);
%         features_pt{nn} = struct('type','cartrbf','pos',pos,'width',width(k),'r',1.0);
    end
    
    if 0
        nn = length(features_pt);
        for k = 1:1
            i = nn + k;
            pos = [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,2)];
            features_pt{i} = struct('type', 'cartebf', 'pos', pos, 'width', mdp_params.feature_radius,...
                                'r', 1.0, 'E', [0.5,0;0,1]);
            features_pt{i+1} = struct('type', 'cartebf', 'pos', pos, 'width', mdp_params.feature_radius,...
                                'r', 1.0, 'E', [0.8,0;0,1.1]);
            features_pt{i+2} = struct('type', 'cartebf', 'pos', pos, 'width', mdp_params.feature_radius,...
                                'r', 1.0, 'E', [0.9,0;0,1.2]);
            features_pt{i+1} = struct('type', 'cartebf', 'pos', pos, 'width', mdp_params.feature_radius,...
                                'r', 1.0, 'E', [2,0;0,1.5]);
            features_pt{i+2} = struct('type', 'cartebf', 'pos', pos, 'width', mdp_params.feature_radius,...
                                'r', 1.0, 'E', [0.5,0;0,0.5]);
        end
    end
    
elseif strcmp(mdp_params.feature_type,'obs') && mdp_params.fixed_pattern == 4
    features_pt = cell(1,1);
    width = linspace(0.3, 1.5, 10);
%    axis1 = [0.5, 0.8, 1];
    centers = [mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end); ...
               mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)];
    axis1 = [0.6, 1];
    
    for l = 1:size(centers,1)
        for k = 1:length(axis1)
            for i = 1:length(width)
                index = (l-1)*length(width)*length(axis1) + (k-1)*length(width) + i;
                features_pt{index} = struct('type', 'cartebf', 'pos', centers(l,:),...
                                'width', width(i), 'r', 1.0, 'E', [axis1(k), 0; 0, 1]);
            end
        end
    end
        
end