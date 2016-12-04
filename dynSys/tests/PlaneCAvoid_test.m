function PlaneCAvoid_test()
% PlaneCAvoid_test()

% Grid
gMin = [-10; -15; 0];
gMax = [25; 15; 2*pi];
gN = [31; 31; 31];
g = createGrid(gMin, gMax, gN);

% Time
tMax = 3;
dt = 0.1;
tau = 0:dt:tMax;

% Vehicle
wMaxA = 1;
wMaxB = 1;
vRangeA = [5 5];
vRangeB = [5 5];
dMaxA = [0 0];
dMaxB = [0 0];

%% Grids and initial conditions
targetR = 5;
data0 = shapeCylinder(g, 3, [0;0;0], targetR);

%% Additional solver parameters
sD.grid = g;
sD.dynSys = PlaneCAvoid([0;0;0], wMaxA, vRangeA, wMaxB, vRangeB, dMaxA, dMaxB);
sD.uMode = 'max';
sD.dMode = 'min';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.keepLast = true;
data = HJIPDE_solve(data0, tau, sD, 'zero', extraArgs);

save(sprintf('%s.mat', mfilename), 'data', 'sD', 'tau', '-v7.3')

end