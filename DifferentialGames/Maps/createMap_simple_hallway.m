function createMap_simple_hallway()
%% create map with half of map as a target and one rectangular obstacle
% obstacle splits frontier of target in half to create two "hallways"

name = 'simple_hallway';

% Grid parameters
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];   % Upper corner of computation domain
N = [41; 41];   % Number of grid points per dimension

% Create grid for target and obstacle data
g = createGrid(grid_min, grid_max, N);

% Target set
target.radius = NaN;
target.center = [3.75; 0];
target.data = shapeRectangleByCorners(g,[0;-5],[5; 5]);

% Static Obstacles
obstacles = shapeRectangleByCenter(g,[0;0],[0.5; 2]);

% Save map
save(['./DifferentialGames/Maps/',name])