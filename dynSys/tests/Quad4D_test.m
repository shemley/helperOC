function Quad4D_test()
% Quad4D_test()

%% Common parameters
targetLower = [-0.5; -inf; -0.5; -inf];
targetUpper = [0.5; inf; 0.5; inf];

% Grid
% gMin = [-2; -3; -3; -3];
% gMax = [5; 3; 4; 3];
gMin = [-4; -3; -4; -3];
gMax = [4; 3; 4; 3];

uMode = 'max';

gN = [65; 61; 65; 61];
% gN = [25; 25; 25; 25];

% Time
tMax = 3;
dt = 0.1;
tau = 0:dt:tMax;

% Vehicle
uMin = -1;
uMax = 1;

%% Grids and initial conditions
g = createGrid(gMin, gMax, gN);
data0 = shapeRectangleByCorners(g, targetLower, targetUpper);

%% Additional solver parameters
sD.grid = g;
sD.dynSys = Quad4D([0;0;0;0], uMin, uMax);
sD.uMode = uMode;

vslice = [2.5 2.5];
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1 0 1 0];
extraArgs.plotData.projpt = vslice;
extraArgs.keepLast = true;
data = HJIPDE_solve(data0, tau, sD, 'zero', extraArgs);

%% Visualize
[g2Dp, data2Dp] = proj(g, data, [0 1 0 1]);

figure
visSetIm(g2Dp, data2Dp);
hold on

vx = -2.5:0.5:2.5;
vy = -2.5:0.5:2.5;
for i = 1:length(vx)
  for j = 1:length(vy)
    vslice = [vx(i) vy(j)];
    [g2Dp, data2Dp] = proj(g, data, [0 1 0 1], vslice);
    visSetIm(g2Dp, data2Dp);
  end
end

save(sprintf('%s.mat', mfilename), 'data', 'sD', 'tau', '-v7.3')

end