function createMap_hallways()
%% create map with two hallways to target in all directions

name = 'hallways';

% Grid parameters
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];   % Upper corner of computation domain
N = [41; 41];   % Number of grid points per dimension

% Create grid for target and obstacle data
g = createGrid(grid_min, grid_max, N);

% Target set
target.radius = 1;
target.center = [0; 0];
target.data = shapeSphere(g, target.center, target.radius);

% Static Obstacles
obsTargetDivide = shapeRectangleByCenter(g,[0;0],[0.5; 2]);
obsTargetTop = shapeRectangleByCorners(g,[-1;1],[1;1.5]);
obsTargetBottom = shapeRectangleByCorners(g,[-1;-1.5],[1;-1]);
obsLeft = shapeRectangleByCorners(g,[-2;-1.5],[-1.5;1.5]);
obsRight = shapeRectangleByCorners(g,[1.5;-1.5],[2;1.5]);
obsTop = shapeRectangleByCorners(g,[-2;2],[2;2.5]);
obsBottom = shapeRectangleByCorners(g,[-2;-2.5],[2;-2]);

% Concatenate and take min along 3rd dimension to combine 2d obstacles
obstacles = cat(3,obsTargetDivide,obsTargetTop,obsTargetBottom,obsLeft,obsRight,obsTop,obsBottom);
obstacles = min(obstacles,[],3);
% Clear individual obstacles
clear obsTargetDivide obsTargetTop obsTargetBottom obsLeft obsRight obsTop obsBottom

% Save map
save(['./DifferentialGames/Maps/',name])