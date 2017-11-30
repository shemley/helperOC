%% Run test case to confirm performance on wheat
function runDiffGameTestCase()

% 1 def pos
% 3 att pos
% 4 horz
% 3 speeds
% cap rad 0.5
% tmax = 2

% map
map = getMap('no_obstacle');

% initial conditions
icGrid.ax = -1.05; 
icGrid.ay = 0; 
icGrid.dx = [-2]; 
icGrid.dy = [2];

dMaxes = 1;

agentInfo.uMax = 1;
agentInfo.captureRadius = 0.5;
horizons = 1;

tMax = 2;

extraArgs.visualize = false;
extraArgs.computeTraj = true;
extraArgs.obstacleType = 'ideal';

for i = 1:length(dMaxes)
    agentInfo.dMax = dMaxes(i);
    [OL,CL,MPC]= diffGameCompare(icGrid, agentInfo, horizons, tMax, map, extraArgs);
    
    filename = sprintf(...
       './DifferentialGames/compareTestData_Test_%s_u%.1f_d%.1f_cr%.1f_t%.1f_n%d.mat',...
       map.name,agentInfo.uMax,agentInfo.dMax,agentInfo.captureRadius,tMax,map.N(1));
    save(filename)
end