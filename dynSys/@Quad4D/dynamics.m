function dx = dynamics(obj, ~, x, u, ~, ~)
% dx = dynamics(obj, t, x, u)
%     Dynamics of the quadrotor

% Dynamics
if iscell(x)
  dx = cell(obj.nx, 1);
  dx{1} = x{2};
  dx{2} = u{1};
  dx{3} = x{4};
  dx{4} = u{2};
else
  dx = obj.A * x + obj.B * u;
end
end

