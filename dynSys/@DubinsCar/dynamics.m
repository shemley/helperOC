function dx = dynamics(obj, ~, x, u, ~, dims)
% Dynamics of the Dubins Car
%    \dot{x}_1 = v * cos(x_3)
%    \dot{x}_2 = v * sin(x_3)
%    \dot{x}_3 = w
%   Control: u = w;
%
% Mo Chen, 2016-06-08

if nargin < 6
  dims = 1:obj.nx;
end

if iscell(x)
  dx = cell(length(dims), 1);
  
  for i = 1:length(dims)
    dx{i} = dynamics_cell_helper(obj, x, u, dims, dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = obj.speed * cos(x(3));
  dx(2) = obj.speed * sin(x(3));
  dx(3) = u;
end
end

function dx = dynamics_cell_helper(obj, x, u, dims, dim)

switch dim
  case 1
    dx = obj.speed * cos(x{dims==3});
  case 2
    dx = obj.speed * sin(x{dims==3});
  case 3
    dx = u;
  otherwise
    error('Only dimension 1-3 are defined for dynamics of DubinsCar!')
end
end