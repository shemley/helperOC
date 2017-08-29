clear all;

% Compile mex file
mex('c version\mexEikonalFMM.cpp')
mex('c version\shortestPathc.cpp')
mex('c version\shortestPathcP.cpp')
numInfty = 1e6;

%% GRID
Nx = 45;

% Create the computation grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

% ----- Load Game -----
% game = 'OLGameModified';
game = 'midTarget_LObs_fastA';
% game = 'midTarget_LObs';
% game = 'nonconvexExample';
run(game);

% Compute value function
u = compute_value(g2D, target2D, velocityd, obs2D, dom_map);
u(isnan(u)) = numInfty;

% Compute path
max_plength = 1000;
pathx = 2*ones(max_plength,1);
pathy = 2*ones(max_plength,1);

% x0 = -1 + 2*rand; y0 = -1 + 2*rand;
% x0 = -0.6; y0 = 0.4;
x0 = -0.8; y0 = 0.2727;
% x0 = -0.8; y0 = 0.8;

pathx(1) = x0; pathy(1) = y0;
speed = velocityd * ones(g2D.N(1), g2D.N(1));
L = g2D.max(1);

tic
% shortestPathc(u, speed, pathx, pathy, L, numInfty);
% path = shortestPath(u,pathx(1),pathy(1),speed,g2D);

P = extractCostates(g2D, u, true);
path = shortestPathP(g2D, P, u, [x0 y0], speed);
pathx = path(1,:);
pathy = path(2,:);
toc

pathx(pathx>1) = [];
pathy(pathy>1) = [];

% Visualize
figure;
contour(g2D.xs{1},g2D.xs{2},obs2D,[0 0],'linecolor','k','linewidth',3)
hold on
contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linecolor','g','linewidth',3)
contour(g2D.xs{1},g2D.xs{2},dom_map,[0 0],'linecolor','k','linewidth',3)

plot(x0, y0, 'b.', 'markersize', 20)
plot(pathx, pathy, 'b-', 'linewidth', 2)
axis square