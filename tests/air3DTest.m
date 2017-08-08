function [data,g]=air3DTest()
%accuracy
accuracy = 'veryHigh';

% time
tMax = 2.8;                  % End time.
dt = 0.25;
tau = 0:dt:tMax;

% Parameters
targetRadius = 5;
velocityA = 5;
velocityB = 5;
inputA = 1;
inputB = 1;

% grid
Nx = 51;
g.min = [  -6; -10;     0 ];
g.max = [ +20; +10; +2*pi ];
g.N = [ Nx; ceil(Nx * (g.max(2) - g.min(2)) / (g.max(1) - g.min(1))); Nx-1 ];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
g.max(3) = g.max(3) * (1 - 1 / g.N(3));
g = createGrid(g.min,g.max,g.N,3);

% data
data0 = shapeCylinder(g, 3, [ 0; 0; 0 ], targetRadius);

% dynamic system
schemeData.dynSys = Air3D([],inputA,inputB,velocityA,velocityB);
schemeData.uMode = 'max';
schemeData.dMode = 'min';
schemeData.accuracy = accuracy;
schemeData.grid = g;

%visualize
extraArgs.visualize = 1;
extraArgs.deleteLastPlot = 1;

% run with air3D plus helper functions (min with zero)
[data.Zero] = HJIPDE_solve(data0, tau, schemeData, 'zeroTEST', extraArgs);

% run with min V over time (min with data from previous time step)
[data.Time] = HJIPDE_solve(data0, tau, schemeData, 'time', extraArgs);

% run with min V with Target (min with original target at each time step)
[data.Target] = HJIPDE_solve(data0, tau, schemeData, 'data0', extraArgs);

% min BRS2BRT (compute BRSs and then compute BRT)
[data.BRS2BRT] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
data.BRS2BRT = min(data.BRS2BRT,[],4);

% run with normal air3D (min with zero)
[data.Original] = air3D(accuracy);

end