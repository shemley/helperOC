function path = shortestPath(u,initX,initY,speed,grid)
% path = ComputeOptimalPath(u,initX,initY,speed,grid)
% Computes optimal paths 
% THIS FUNCTION ASSUMES GRID IS IDENTICAL IN X and Y DIRECTIONS!
% ALSO ASSUMES CONSTANT SPEED (POSITION INVARIANT)
%
% INPUTS:
%   u -             value function (0 level set = target)
%   (initX,initY) - initial point
%   speed -         speed profile
%   grid -          2D grid structure
%
% OUTPUT
%   path(1,:) -     x coordinates of the path
%   path(2,:) -     y coordinates of the path
%
% Adapted from Haomiao Huang, Zhengyuan Zhou
% Mo Chen, 2014-02-12

% tic
%
% Unpack grid
checkGrid(grid);
N = grid.N(1);
L = grid.max(1);
numInfty = 1e6;
u(isnan(u)) = numInfty;

% If speed profile is a scalar, then change it to match size of grid
if numel(speed) == 1, speed = speed*ones(N); end

% If size of speed profile does not match that of grid, resample
if ~(size(speed) == [N N])
    speedvx = linspace(1,size(speed,1),N);
    speedvy = linspace(1,size(speed,2),N);
    [speedVx, speedVy] = ndgrid(speedvx, speedvy);
    speed = interpn(speed, speedVx, speedVy);
end

max_plength = 10000;
pathx = 2*ones(max_plength,1);
pathy = 2*ones(max_plength,1);
pathx(1) = initX;
pathy(1) = initY;

shortestPathc(u, speed, pathx, pathy, L, numInfty);
pathx(pathx>1) = [];
pathy(pathy>1) = [];

path = [pathx pathy]';

end
