function diffGame_closed_loop()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;

%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [-5; -5; -5; -5]; % Lower corner of computation domain
grid_max = [5; 5; 5; 5];     % Upper corner of computation domain
N = [41; 41; 41; 41];        % Number of grid points per dimension
pdDims  = [];                % no periodic dimensions. 
                             % Create function in dyn sys to return periodic dims?
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
targetCenter = [0; 0; 0; 0];
data0 = shapeCylinder(g, [3 4], targetCenter, R); % target set for attacker is a circle, ignore defender position
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
% control limit (attacker)
uMax = 1;
% do dStep1 here
% disturbance limit (defender)
dMax = uMax; % apply same control limit to defender

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here
% attacker seeks to maximize cost function
dMode = 'max';

% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData

% Use backward reachable tube for differential game. We care about all
% points that can reach the target within a time limit, not just point that
% reach the target at exactly that time
minWith = 'zero';


%% Pack problem parameters

% Define dynamic system
% obj = KinVehicleND2Agent(x, Max, speed, dMax)
dynSys = KinVehicleND2Agent([0,0,0,0], uMax, dMax); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dynSys;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
%do dStep4 here


%% If you have obstacles, compute them here
% avoid set
% obstacles = avoidSetMultiAgent(grid, nagents, ndims, radii, ignoreDims)
captureRadius = 1;
nagents = 2;
ndims = 2;
ignoreDims = [];
obstacles = avoidSetMultiAgent(g,captureRadius,nagents,ndims,ignoreDims);

% Set obstacles
HJIextraArgs.obstacles = obstacles;

%% Compute value function

HJIextraArgs.visualize = false; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

% save open_loop_test_square.mat

%% Compute optimal trajectory from some initial state
if compTraj
  pause
  
  xinit = [-1, 1, -2, 3];
    
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dynSys.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = dMode; %set if disturbance wants to min or max
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    agentDim = [1 1 0 0];
    TrajextraArgs.projDim = agentDim;
    
    % Show disturbance
    distDim = [0 0 1 1];
    TrajextraArgs.distDim = distDim;
    
    % Show target set
    TrajextraArgs.targetData = data0;  
    TrajextraArgs.targetCenter = targetCenter;   
    
    % Show obstacle set
    TrajextraArgs.obstacleData = obstacles;
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,5);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dynSys, TrajextraArgs);
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end