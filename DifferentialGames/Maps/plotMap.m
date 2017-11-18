function f = plotMap(map,fig)
% plot a game map from a map structure
% Inputs:
%   map - map structure
%       .name
%       .min
%       .max
%       .N
%       .g
%       .target
%           .radius
%           .center
%           .data
%       .obstacles - might not be present
%   fig - handle for figure name (optional)

if nargin < 2
    f = figure;
else
    f = figure(fig);
end

% Plot grid
% visGrid(map.g);

% Plot target set
visSetIm(map.g, map.target.data, 'g');

% Plot obstacles if present
if isfield(map, 'obstacles')
    visSetIm(map.g, map.obstacles, 'b');
end




