function inObstacle = isInObstacle(map, x)
% function to determine if a vector x in an obstacle for the input map

inObstacle = false;

if isfield(map,'obstacles')
    inObstacle = (eval_u(map.g, map.obstacles, x) <= 0);
end