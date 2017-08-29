function path = ComputeOptimalPath(u,initX,initY,speed,grid)
% path = ComputeOptimalPath(u,initX,initY,speed,grid)
% Computes optimal paths 
% THIS FUNCTION ASSUMES GRID IS IDENTICAL IN X and Y DIRECTIONS!
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
% Adapted from Haomiao Huang, Zhengyuan Zho
% Mo Chen, 2013-06-07


%
% Unpack grid
checkGrid(grid);
N = grid.N(1);
dx = grid.dx(1);

ds = 0.5; % step size is 0.5*dx

TOL = 0.5*dx;

% If speed profile is a scalar, then change it to match size of grid
if numel(speed) == 1, speed = speed*ones(N); end

% Change NaN values to infinity
u(isnan(u)) = 1e6;

maxPathLength = 100000;

% initialize vector
path_x = zeros(1,maxPathLength);
path_y = zeros(1,maxPathLength);

% initial points
[path_x(1),path_y(1)] = xy2inds(initX,initY,grid);

flag = 0;

for k =1:maxPathLength
    % round to nearest pixel to compute gradient
    ex0 = round(path_x(k));
    ey0 = round(path_y(k));
    
    ue0 = u(ex0,ey0);
    
    % check tolerance from final positions
    if (ue0 < TOL)
        path = [path_x(1:k); path_y(1:k)];
        path = 2/(N-1)*(path-1)-1;
        flag = 1;
        break;
    end
       
    % Compute gradient
    % One-sided differencing on the boundary
    % Central differencing in the interior of grid
    if (ex0 == 1)
        uexp = u(ex0+1, ey0);
        uexm = u(ex0, ey0);
    elseif (ex0 == N)
        uexp = u(ex0, ey0);
        uexm = u(ex0-1,ey0);
    else
        uexp = u(ex0+1, ey0);
        uexm = u(ex0-1, ey0);
    end
    
    if (ey0 == 1)
        ueyp = u(ex0,ey0+1);
        ueym = u(ex0,ey0);
    elseif (ey0 == N)
        ueyp = u(ex0,ey0);
        ueym = u(ex0,ey0-1);
    else
        ueyp = u(ex0,ey0+1);
        ueym = u(ex0,ey0-1);
    end
    
    if uexp < uexm, e_gx = uexp - ue0;
    else            e_gx = ue0 - uexm;
    end
    
    if ueyp < ueym, e_gy = ueyp - ue0;
    else            e_gy = ue0 - ueym;
    end
    
    ngrad_e = sqrt(e_gx^2 + e_gy^2);

    % update new value
    path_x(k+1) = path_x(k) - speed(ex0,ey0)*ds * e_gx /ngrad_e;
    path_y(k+1) = path_y(k) - speed(ex0,ey0)*ds * e_gy /ngrad_e;
    
%     figure(3),hold on;
%     plot((path_x(k+1)-1)*2/(N-1)-1,(path_y(k+1)-1)*2/(N-1)-1,'b+','MarkerSize',5);
%     drawnow;
end

if (flag == 0)
    disp(['Error: can not reach target within ',num2str(maxPathLength), ' steps!']);
    path = 0;
end
