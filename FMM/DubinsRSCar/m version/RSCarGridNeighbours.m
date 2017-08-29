% Given accepted nodes in Reeds-Shepp's car problem, find narrow-band nodes
% Run this once after changing grid
%% Grid
Nx = 21; Ny = Nx; Ntheta = Nx;
x = linspace(-1,1,Nx);
y = linspace(-1,1,Ny);
theta = linspace(0,pi,Ntheta);

dx = x(2)-x(1);
dy = y(2)-y(1);
dtheta = theta(2)-theta(1);

[X Y Theta] = meshgrid(x,y,theta);

% Speed profile
c = ones(size(X));

%% precompute neighbours for all nodes
% neighbours stores the nodes required to compute node i,j,k as a function
% of theta
global neighbours
neighbours = cell(3,2,Ntheta);

global endPts
endPts = cell(3,2,Ntheta);

global forward
global turn

traj = cell(3,2);

for k = 1:Ntheta % for every theta node
    for n = 1:2 % forward and backward
        switch n
            case 1
                forward = -1;
            case 2
                forward = 1;
        end

        for m = 1:3    % turn left, turn right, go straight
            switch m
                case 1
                    turn = -1;
                case 2
                    turn = 0;
                case 3
                    turn = 1;
            end
            
            switch turn
                case 0
                    if cos(theta(k))
                        tspan = [0 min([cos(theta(k)) sin(theta(k))])];
                    end
                otherwise
                    tspan = [0 dtheta];
                    [time traj{m,n}] = ode45(@RSCarDynamics, tspan, [0 0 theta(k)]);

                    xl = floor(traj{m,n}(end,1)/dx); xu = ceil(traj{m,n}(end,1)/dx);
                    yl = floor(traj{m,n}(end,2)/dy); yu = ceil(traj{m,n}(end,2)/dy);

                    neighbours{m,n,k} = [xl yl -turn;
                        xl yu -turn;
                        xu yl -turn;
                        xu yu -turn];
                    
                    endPts{m,n,k} = [traj{m,n}(end,1)/dx traj{m,n}(end,2)/dy];

            end
        end
    end
end


run 'RSCar'
