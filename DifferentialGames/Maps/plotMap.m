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

hold on

% Plot grid
% visGrid(map.g);

% Plot grid boundary
bndryPoints = [map.grid_min, [map.grid_min(1); map.grid_max(2)],...
             map.grid_max,[map.grid_max(1); map.grid_min(2)],map.grid_min];
plot(bndryPoints(1,:),bndryPoints(2,:),'k--')

% Plot target set
visSetIm(map.g, map.target.data, 'g');

% Plot obstacles if present
if isfield(map, 'obstacles')
    visSetIm(map.g, map.obstacles, 'k');
end

hold off



