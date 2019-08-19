function output = threedprojection(a,b,c,d, u,v,w)

% p = point;
% s = plane;
% output = cross([a, b, c, d]', [u, v, w, 1]) - (a*u + ...
output = [a, b, c, d]' * [u, v, w, 1] - (a*u + ...
    b*v + c*w + d)*eye(4); 
