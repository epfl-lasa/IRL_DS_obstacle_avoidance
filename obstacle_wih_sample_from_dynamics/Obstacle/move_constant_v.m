function y = move_constant_v(x)
    % y = [10-x(1,:);2.2-x(2,:)];
    % y = [10/size(x,2)*ones(1,size(x,2));...
    %     zeros(1,size(x,2))];
    y = [ 10 - x(1,:) % 5;...%10 - x(1,:);...
    %    zeros(1,size(x,2))];
         4.2 - x(2,:)];
    y = y * 0.01;
    % y = [0.3*ones(1,size(x,2));...
    %     zeros(1,size(x,2))];
end