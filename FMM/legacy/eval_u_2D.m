function v = eval_u_2D(u,X,Y,g)
% v = eval_u(u,X,Y,grid)
% evaluates the value function at points (X,Y)
%
% INPUTS:
%   u:  value function - u(X,Y)
%   X:  vector of x values
%   Y:  vector of y values
%   grid:
%
% OUTPUT
%   v:  value at points X and Y
%
% Mo Chen, 2013-06-07
%

checkGrid(g); % Check grid compatibility

if length(X)~=length(Y)
    error('# of x coordinates must be the same as # of y coordinates')
end

Xv = g.vs{1};
Yv = g.vs{2};

v = interpn(Xv,Yv,u,X,Y);
if any(isnan(v))
    v(isnan(v)) = interpn(Xv,Yv,u,X(isnan(v)),Y(isnan(v)), 'nearest');
end

if any(isnan(v))
    v(isnan(v)) = inf;
end
end