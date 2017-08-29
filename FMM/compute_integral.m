function area = compute_integral(grid,u)
% area = compute_integral(grid,u)
% Computes the integral under a surface or curve using trapezoid rule
%
% Inputs:
%   grid    - 2D grid structure
%   u       - function to integrate
%
% Output:
%   area    - integral
% 
% Mo Chen, 2014-02-12
%

dx = diff(grid.vs{1})'; dx = dx(1:end-1);
dy = diff(grid.xs{2},1,2);

areax = 0.5*sum(dy.*(u(1:end,1:end-1) + u(1:end, 2:end))); % Integral of u(x,y) dy
area = 0.5*sum(dx.*(areax(1:end-1) + areax(2:end))); % Integral of areax(x) dx
end