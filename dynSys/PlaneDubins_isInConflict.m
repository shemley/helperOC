function [conflict, sVal] = PlaneDubins_isInConflict(x_evade, x_other, g, data)

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
sVal = eval_u(g, data, xr);
conflict = sVal < 0;
end