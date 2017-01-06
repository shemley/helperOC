function tEarliest = find_earliest_BRS_ind(g, data, x, upper, lower)
% Determine the earliest time that the current state is in the reachable set
% Binary search

clns = repmat({':'}, 1, g.dim);
small = 1e-4;

while upper > lower
  tEarliest = ceil((upper + lower)/2);
  valueAtX = eval_u(g, data(clns{:}, tEarliest), x);
  
  if valueAtX < small
    % point is in reachable set; eliminate all lower indices
    lower = tEarliest;
  else
    % too late
    upper = tEarliest - 1;
  end
end

tEarliest = upper;
end