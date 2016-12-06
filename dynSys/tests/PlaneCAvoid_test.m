function PlaneCAvoid_test()
% PlaneCAvoid_test()
%     Tests the PlaneCAvoid class; requires the level set toolbox, which can be
%     found at https://www.cs.ubc.ca/~mitchell/ToolboxLS/

%% Grid
% Choose this to be just big enough to cover the reachable set
gMin = [-10; -15; 0];
gMax = [25; 15; 2*pi];
gN = [65; 65; 65];
g = createGrid(gMin, gMax, gN);

%% Time
% Choose tMax to be large enough for the set to converge
tMax = 3;
dt = 0.1;
tau = 0:dt:tMax;

%% Vehicle parameters
% Maximum turn rate (rad/s)
wMaxA = 1;
wMaxB = 1;

% Speed range (m/s)
vRangeA = [5 5];
vRangeB = [5 5];

% Disturbance (see PlaneCAvoid class)
dMaxA = [0 0];
dMaxB = [0 0];

%% Initial conditions
targetR = 5; % collision radius
data0 = shapeCylinder(g, 3, [0;0;0], targetR);

%% Additional solver parameters
sD.grid = g;
sD.dynSys = PlaneCAvoid([0;0;0], wMaxA, vRangeA, wMaxB, vRangeB, dMaxA, dMaxB);
sD.uMode = 'max';
sD.dMode = 'min';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.keepLast = true;

%% Call solver and save
data = HJIPDE_solve(data0, tau, sD, 'zero', extraArgs);
save(sprintf('%s.mat', mfilename), 'data', 'sD', 'tau', '-v7.3')

end