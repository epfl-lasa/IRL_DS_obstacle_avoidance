function o = project3(s, p)

a = s(1); b = s(2); c = s(3); d = s(4);
u = p(1); v = p(2); w = p(3);
% syms t
% eqn = a*(a*t+u) + b*(b*t+v) + c*(c*t+w) + d == 0;

% solx = solve(eqn, t);

solx = -(a*u + b*v + c*w +d)/(a^2 + b^2 + c^2);

x = a*solx+u;
y = b*solx+v;
z = c*solx+w;

o = [x,y,z];

