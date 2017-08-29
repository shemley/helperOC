function path = shortestPathP(grid, P, u, x, speed)
% path = shortestPathP(grid, P, x, speed)
% Computes optimal paths 
%
% INPUTS:
%   P -             costate information (0 level set = target)
%   u -             value function
%   x = (initX,initY) - initial point
%   speed -         speed profile
%   grid -          2D grid structure
%
% OUTPUT
%   path(:,1) -     x coordinates of the path
%   path(:,2) -     y coordinates of the path
%
% Adapted from Haomiao Huang, Zhengyuan Zhou
% Mo Chen, 2014-10-10

checkGrid(grid);
N = grid.N(1);
L = grid.max(1);
numInfty = 1e6;
u(isnan(u)) = numInfty;

% If speed profile is a scalar, then change it to match size of grid
if numel(speed) == 1, speed = speed*ones(size(u)); end

% If size of speed profile does not match that of grid, resample
if ~(size(speed) == size(u))
    speedvx = linspace(1,size(speed,1),size(u,1));
    speedvy = linspace(1,size(speed,2),size(u,2));
    [speedVx, speedVy] = ndgrid(speedvx, speedvy);
    speed = interpn(speed, speedVx, speedVy, 'nearest');
end

max_plength = 10000;
pathx = 2*ones(max_plength,1);
pathy = 2*ones(max_plength,1);
pathx(1) = x(1);
pathy(1) = x(2);

% keyboard
shortestPathcP(u, speed, pathx, pathy, P{1}, P{2}, L, numInfty);
pathx(pathx>1) = [];
pathy(pathy>1) = [];

path = [pathx pathy]';
% keyboard
% while t<tMax
%     
%     speedij = eval_u(grid, speed, x);
%     
%     fprintf('t=%f, speedij=%f\n',t, speedij);
%     dt = 0.5*mean(grid.dx)/speedij;
%     
%     dirn = shortestPathDirectionP(grid, P, x);
%     x = x + speedij*dirn*dt;
%     path = cat(1, path, x);
%     
%     t = t+dt;
%     tau = cat(1, tau, t);
%     
%     if eval_u(grid,u,x) < small
%         disp('Destination reached!')
%         return;
%     end
% end

% disp('Maximum time reached!')

end
