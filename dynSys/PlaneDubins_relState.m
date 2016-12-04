function xr = PlaneDubins_relState(x_evade, x_other)

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

xr = x_other - x_evade;
xr(:,3) = wrapTo2Pi(xr(:,3));
xr(:,1:2) = rotate2D(xr(:,1:2), -x_evade(:,3));

end