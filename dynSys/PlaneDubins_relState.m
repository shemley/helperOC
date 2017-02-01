function xr = PlaneDubins_relState(x_evade, x_other)
% xr = PlaneDubins_relState(x_evade, x_other)
%     Computes the relative states of two Plane or DubinsCar objects.
%
% Inputs:
%     x_evade: state of the vehicle that performs avoidance. This state will be
%              considered to be at the origin (0,0) position and 0 heading
%     x_other: state of the vehicle not performing avoidance. This state will be
%              translated and rotated such that x_evade is at the origin
%
% Output:
%     xr: relative state

if size(x_evade,2) ~= 3
  x_evade = x_evade';
end

if size(x_evade,2) ~= 3
  error('x_evade must have 3 rows or 3 columns!')
end

if size(x_other,2) ~= 3
  x_other = x_other';
end

if size(x_other,2) ~= 3
  error('x_other must have 3 rows or 3 columns!')
end

if size(x_evade,1) ~= size(x_other,1)
  error('x_evade must have the same number of entries as x_other!')
end

% Translate so that x_evade is at origin
xr = x_other - x_evade;
xr(:,3) = wrapTo2Pi(xr(:,3));

% Rotate coordinates so x_evade has 0 heading
xr(:,1:2) = rotate2D(xr(:,1:2), -x_evade(:,3));

end