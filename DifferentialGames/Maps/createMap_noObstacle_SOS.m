function createMap_noObstacle_SOS()
%% create no obstacle map with SOS target

name = 'no_obstacle_SOS';

% Grid parameters
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];   % Upper corner of computation domain
N = [41; 41];   % Number of grid points per dimension

% Create grid for target and obstacle data
g = createGrid(grid_min, grid_max, N);

% Target set
target.radius = 1;
target.center = [0; 0];
target.data = shapeRectangleByCenter(...
                      g, target.center, repmat(2*target.radius/sqrt(2),2,1));

% Static Obstacles
% none

% Save map
save(['./DifferentialGames/Maps/',name])