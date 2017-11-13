function [OL,CL,MPC]= diffGameCompare(icGrid, agentInfo, horizons, tMax, visualize, dataCL)
% Runs solvers for closed loop, open loop, and MPC approaches for a
% differential reach-avoid game between two single integrators for time
% tMax and an execution horizon of horizon (for MPC). 
%
% Inputs:
%   icGrid - vectors defining a 4D grid of initial conditions to test.
%            Results in a huge number of cases. Recommend fixing attacker
%            or defender
%       .ax - vector of attacker initial x positions to test
%       .ay - vector of attacker initial y positions to test
%       .dx - vector of attacker initial x positions to test
%       .dy - vector of attacker initial y positions to test
%   agentInfo - parameters to define the 2 agents (attacker and defender)
%       .captureRadius - minimum distance between agents if attacker is not
%                        captured (defaults to 1)
%       .uMax          - maximum velocity of the attacker (defaults to 1)
%       .dMax          - maximum velocity of the defender (defaults to 1)
%   horizon - Time between re-planning for MPC approach. Given in units of
%             time, but rounded to an integer number of time steps.
%   tMax - total simulation time (defaults to 2)
%   visualize - set to true to visualize results (defaults to false)
%
% Outputs:
%   OL - open loop results
%   CL - closed loop results
%   MPC - MPC results
%       .horizon = actual horizon used for MPC

%% Process input

% Process Initial condition grid data
if isfield(icGrid,'ax') && isfield(icGrid,'ay') &&...
   isfield(icGrid,'dx') && isfield(icGrid,'dy')
    
    [AX,AY,DX,DY] = ndgrid(icGrid.ax, icGrid.ay, icGrid.dx, icGrid.dy);
else
    error('x and y positions of attacker and defender required in separate vectors')
end

% Capture radius
if isfield(agentInfo,'captureRadius')
    captureRadius = agentInfo.captureRadius;
else
    captureRadius = 1;
end

% Max attacker velocity
if isfield(agentInfo,'uMax')
    uMax = agentInfo.uMax;
else
    uMax = 1;
end

% Max defender velocity
if isfield(agentInfo,'dMax')
    dMax = agentInfo.dMax;
else
    dMax = 1;
end

% time vector
t0 = 0;
dt = 0.05;

if nargin < 4
    tMax = 2;
end

% Ensure tMax and horizon are multiples of dt
tMax = ceil(tMax/dt)*dt;
numHorizonSteps = unique(floor(horizons./dt));
horizons = numHorizonSteps.*dt;
fprintf('\nUsing end time of %.2f and horizon of %.2f\n',tMax,horizons(1))

% Time vector
tau = t0:dt:tMax;

% Visualization input
if nargin < 5
    visualize = false;
end

% Allow stores CL data to be input to avoid re-computing
if nargin < 6
    computeCL = true;
else
    computeCL = false;
end


%% General parameters (OL and CL)
% Grid parameters
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];   % Upper corner of computation domain
N = [41; 41];        % Number of grid points per dimension
pdDims  = [];        % no periodic dimensions. 

% Target set
targetRadius = 1;
targetCenter = [0; 0];

% Control modes
uMode = 'min'; % Attacker trying to min value function
dMode = 'max'; % Attacker trying to max value function

% Use backward reachable tube to include all points that can reach target
% within tMax (not at tMax exactly)
minWith = 'zero';

% Extra arguments for computing optimal trajectories
TrajextraArgs.uMode = uMode; %set if control wants to min or max
TrajextraArgs.dMode = dMode; %set if disturbance wants to min or max
TrajextraArgs.visualize = false; % do not plot

%% Closed Loop (CL) parameters
% Colons for taking time slices of data (e.g. data(clns,time))
clnsCL = repmat({':'}, 1, 4);

% Create CL grid using grid parameters defined above concatenated into 4D
% for attacker and defender positions
gCL = createGrid([grid_min;grid_min],[grid_max; grid_max],[N;N],pdDims);

% Target set (concatenate into 4D
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
ignoreDims = [3,4]; % target is only in attacker dimensions
targetDataCL = ...  % target set for attacker is a circle, ignore defender
    shapeCylinder(gCL,ignoreDims,[targetCenter;targetCenter],targetRadius); 

% Define dynamic system
% obj = KinVehicleND2Agent(x, Max, speed, dMax)
dynSysCL = KinVehicleND2Agent([0,0,0,0], uMax, dMax);

% Put grid and dynamic systems into schemeData
schemeDataCL.grid = gCL;
schemeDataCL.dynSys = dynSysCL;
schemeDataCL.accuracy = 'high'; %set accuracy
schemeDataCL.uMode = uMode;
schemeDataCL.dMode = dMode;

% Solver extra parameters
HJIextraArgsCL.visualize = false; %show plot

% Obstacles: Avoid set
% obstacles = avoidSetMultiAgent(grid, nagents, ndims, radii, ignoreDims)
nagents = 2;
numDims = 2;
ignoreDims = [];
obstaclesCL = ...
    avoidSetMultiAgent(gCL,captureRadius,nagents,numDims,ignoreDims);

% Set obstacles
HJIextraArgsCL.obstacles = obstaclesCL;

%% Open Loop (OL) parameters
% Colons for taking time slices of data (e.g. data(clns,time))
clnsOL = repmat({':'}, 1, 2);

% Create OL grid using grid parameters defined above
gOL = createGrid(grid_min, grid_max, N, pdDims);

% data0 = shapeCylinder(grid,ignoreDims,center,radius)
targetCenter = [0; 0];
% target set for attacker is a circle
targetDataOL = shapeSphere(gOL, targetCenter, targetRadius); 

% Define dynamic system
% obj = KinVehicleND(x, uMax)
dynSysOL = KinVehicleND([0,0], uMax);

% Put grid and dynamic systems into schemeData
schemeDataOL.grid = gOL;
schemeDataOL.dynSys = dynSysOL;
schemeDataOL.accuracy = 'high'; %set accuracy
schemeDataOL.uMode = uMode;

% Solver extra parameters
HJIextraArgsOL.visualize = false; %show plot

%% Compute CL value function

if computeCL
    % %[data, tau, extraOuts] = ...
    % % HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
    [dataCL, tauCL, ~] = ...
      HJIPDE_solve(targetDataCL, tau, schemeDataCL, minWith, HJIextraArgsCL);
end
% else dataCL given as input
  
% For trajectory computation:
%flip data time points so we start from the beginning of time
dataTrajCL = flip(dataCL,ndims(dataCL));

%% Set up input for MPC solver
% OL parameter structure
OLin.g = gOL;
OLin.targetData = targetDataOL;
OLin.dynSys = dynSysOL;
OLin.schemeData = schemeDataOL;
OLin.minWith = minWith;
OLin.HJIextraArgs = HJIextraArgsOL;
OLin.TrajextraArgs = TrajextraArgs;
OLin.clns = clnsOL;

% CL parameter structure
CLin.g = gCL;
CLin.data = dataCL;
CLin.dataTraj = dataTrajCL;
CLin.dynSys = dynSysCL;
CLin.obstacles = obstaclesCL;
CLin.HJIextraArgs = HJIextraArgsCL;
CLin.TrajextraArgs = TrajextraArgs;
CLin.clns = clnsCL;

%% Loop through Defender positions
for idx = 1:size(DX,3)
  for idy = 1:size(DY,4)
    % Set defender position
    defenderPos = [DX(1,1,idx,idy); DY(1,1,idx,idy)]; 
      
    %% Compute OL obstacles
    % Define Obstacles
    obsCenter  = defenderPos;   
    obstaclesOL = getOpenLoopAvoidSet(gOL,obsCenter,captureRadius,dMax,tau);
        
    % Set obstacles
    HJIextraArgsOL.obstacles = obstaclesOL;

    %% Compute OL value function
    %[data, tau, extraOuts] = ...
    % HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
    [dataOL, tauOL, ~] = ...
      HJIPDE_solve(targetDataOL, tau, schemeDataOL, minWith, HJIextraArgsOL);
      
    % For trajectory computation:
    %flip data time points so we start from the beginning of time
    dataTrajOL = flip(dataOL,ndims(dataOL));

    %% Loop through attacker positions
    for iax = 1:size(AX,1)
      for iay = 1:size(AY,2)
        % Set attacker position  
        attackerPos = [AX(iax,iay,idx,idy); AY(iax,iay,idx,idy)];  
        % Set joint inital position
        initPos = [attackerPos; defenderPos];
        % Set agentInfo for MPC solver
        agentInfo.initPos = initPos;
        
        
        %% Compute OL optimal trajectory
        %value = eval_u(g, data, x)
        valueOL = eval_u(gOL,dataOL(clnsOL{:},length(tau)),attackerPos);

        % find optimal trajectory
        dynSysOL.x = attackerPos; %set initial state

        % [traj, traj_tau] = ...
        % computeOptTraj(g, data, tau, dynSys, extraArgs)
        [trajOL, traj_tauOL] = ...
          computeOptTraj(gOL, dataTrajOL, tauOL, dynSysOL, TrajextraArgs);

        %% Compute CL optimal trajectory
        %value = eval_u(g, data, x)
        valueCL = eval_u(gCL,dataCL(clnsCL{:},length(tau)),initPos);

        % find optimal trajectory
        dynSysCL.x = initPos; %set initial state

        % [traj, traj_tau] = ...
         % computeOptTraj(g, data, tau, dynSys, extraArgs)
        [trajCL, traj_tauCL] = ...
          computeOptTraj(gCL, dataTrajCL, tauCL, dynSysCL, TrajextraArgs);

        %% Loop through horizons
        for ihz = 1:length(numHorizonSteps)            
          %% Compute MPC optimal trajectory  
          [trajMPC, BRSdata, obsData] = ...
            diffGameSolveMPC(agentInfo,numHorizonSteps(ihz),tau,CLin,OLin);
          % diffGameSolveMPC(agentInfo, horizonSteps, tau, CL, OL, mapData)
        end
      end
    end
  end
end
        
%% Visualize
if visualize
    figure;
    plot(trajCL(1,:),trajCL(2,:),'b.-')
    hold on
    plot(trajCL(3,:),trajCL(4,:),'c.-')
    plot(trajOL(1,:),trajOL(2,:),'k.-')
    plot(trajMPC(1,:),trajMPC(2,:),'r.-')
    plot(trajMPC(3,:),trajMPC(4,:),'m.-')
    hold off
end

end