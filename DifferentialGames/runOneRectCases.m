%% Run grid of points on no obstacle map for varying speeds

function runOneRectCases()

% 1 def pos
% 3 att pos
% 4 horz
% 3 speeds
% cap rad 0.5
% tmax = 2

% map
map = getMap('one_rectangle');

% initial conditions
icGrid.ax = [-2]; 
icGrid.ay = [0]; 
icGrid.dx = [-2]; 
icGrid.dy = [2];

dMaxes = [0.5; 1; 2; 2.5; 3; 4; 5];

agentInfo.uMax = 1;
agentInfo.captureRadius = 0.5;
horizons = [0.10; 0.25 ;0.5; 1];

tMax = 2;

extraArgs.visualize = false;
extraArgs.computeTraj = true;
extraArgs.obstacleType = 'ideal';

for i = 1:length(dMaxes)
    agentInfo.dMax = dMaxes(i);
    [OL,CL,MPC]= diffGameCompare(icGrid, agentInfo, horizons, tMax, map, extraArgs);
    
    filename = sprintf(...
       './DifferentialGames/compareTestData_OneRectGen_%s_u%.1f_d%.1f_cr%.1f_t%.1f_n%d.mat',...
       map.name,agentInfo.uMax,agentInfo.dMax,agentInfo.captureRadius,tMax,map.N(1));
    save(filename)
end