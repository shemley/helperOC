function [conflict, sVal] = PlaneDubins_isInConflict(x_evade, x_other, g, data)
% [conflict, sVal] = PlaneDubins_isInConflict(x_evade, x_other, g, data)
%     Checks whether two sets of planes are in conflict
%
% Inputs: 
%     x_evade: states of planes which are performing avoidance
%     x_other: states of planes which are not performing avoidance
%     g:       grid structure
%     data:    safety value function (look-up table computed from 
%              PlaneCAvoid_test.m)
%
% Outputs:
%     conflict: boolean that indicates whether each plane pair is in conflict
%     sVal:     "safety value" -- interpolate value on the look-up table

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

xr = PlaneDubins_relState(x_evade, x_other);

if any(xr(1:2) < g.min(1:2)) || any(xr(1:2) > g.max(1:2))
  % No conflict if out of grid
  sVal = 1e6;
  conflict = false;
else
  % Conflict if "safety value" is negative
  sVal = eval_u(g, data, xr);
  conflict = sVal < 0;
end


end