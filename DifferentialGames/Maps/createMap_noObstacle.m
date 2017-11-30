function createMap_noObstacle()
%% create standard no obstacle map

name = 'no_obstacle';

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
% none

% Save map
save(['./DifferentialGames/Maps/',name])