function dx = dynamics(obj, ~, x, u, d, ~)
% Dynamics of 2 ND kinematic vehicles
%    Agent 1:
%    \dot{x}_1 = u_x
%    \dot{x}_2 = u_y
%        u = (u_x, u_y)
%    Agent 2:
%    \dot{x}_1 = d_x
%    \dot{x}_2 = d_y
%        d = (d_x, d_y)

dx = [u;d];

end