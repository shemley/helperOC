function inTarget = isInTarget(map, x)
% function to determine if a vector x in the target for the input map

inTarget = (eval_u(map.g, map.target.data, x) <= 0);