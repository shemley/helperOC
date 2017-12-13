function onMap = isOnMap(map, x)
% Function to check if a vector x is on the map. Returns true if x is on
% the map

% check x dims
if ~all(size(map.grid_min) == size(x))
    if all(size(map.grid_min) == size(x'))
        x = x';
    else
        error('X dimensions must match map dimensions')
    end
end

% Check that x falls inside the map min and max in every dimension
onMap = all(and((x >= map.grid_min), (x <= map.grid_max)));