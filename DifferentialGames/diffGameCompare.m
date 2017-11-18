function [OL,CL,MPC]= diffGameCompare(icGrid, agentInfo, horizons, tMax, visualize, savedCL)
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
    % Compute and save results
    computeCL = true;
    saveCLresult = true;
else
    % Use input data provided that it is for the same parameters
    computeCL = false;
    saveCLresult = false;
    
    % Check that the parameters are provided along with the input CL data
    if isfield(savedCL,'uMax') && isfield(savedCL,'dMax') && ...
       isfield(savedCL,'captureRadius') && isfield(savedCL,'tMax')
        
        % Check that CL data matches the input parameters
        if savedCL.uMax == uMax && savedCL.dMax == dMax &&...
           savedCL.captureRadius == captureRadius && ...
           savedCL.tMax == tMax
    
            % Use saved data
            dataCL = savedCL.data;
        else
           error('Saved CL data doesn''t match input parameters.')
        end
    else
      error('Saved CL data must be in a struct with corresponding parameters.')
    end
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

% Extra trajectory arguments for computing only one step of trajectory
TrajextraArgsOneStep = TrajextraArgs;
TrajextraArgsOneStep.trajPoints = 2;

% Tolerance for arrival in target/obstacle sets
small = 1e-4;

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
% obj = KinVehicleND2Agent(x, uMax, dMax)
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
    % Print message
    tic
    fprintf('Computing Closed Loop solution...\n')
    
    % %[data, tau, extraOuts] = ...
    % % HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
    [dataCL, tauCL, ~] = ...
      HJIPDE_solve(targetDataCL, tau, schemeDataCL, minWith, HJIextraArgsCL);
  
    % Print message
    fprintf('Closed Loop solution computed.\t%.2f seconds\n',toc)
else    
    % else dataCL given as input  
    % Print message
    fprintf('Using pre-computed Closed Loop solution.\n')
end


% Save CL results and parameters in struct if requested
if saveCLresult
    % Print message
    tic
    fprintf('Saving Closed Loop solution...\n')
    
    % Build struct
    savedCL.data = dataCL;
    savedCL.uMax = uMax;
    savedCL.dMax = dMax;
    savedCL.captureRadius = captureRadius;
    savedCL.tMax = tMax;
    savedCL.N = N;
    
    clFilename = sprintf(...
       './DifferentialGames/CLresults_u%.1f_d%.1f_cr%.1f_t%.1f_n%d.mat',...
                                   uMax,dMax,captureRadius,tMax,N(1));
    save(clFilename,'savedCL')
    
    % Print message
    fprintf('Closed Loop solution saved.\t%.2f seconds\n',toc)
end
  
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

%% Set up output structs
% Closed loop
CL.AX = AX;
CL.AY = AY;
CL.DX = DX;
CL.DY = DY;
CL.Values = nan(size(AX));
CL.trajectories = cell(size(AX));
CL.data = dataCL;
CL.obstacles = obstaclesCL;
CL.targetData = targetDataCL;

% Open loop
OL.AX = AX;
OL.AY = AY;
OL.DX = DX;
OL.DY = DY;
OL.Values = nan(size(AX));
OL.trajectories = cell(size(AX));
OL.data = cell(size(AX));
OL.obstacles = cell(size(AX));
OL.targetData = targetDataOL;

% MPC
MPC.horizons = horizons;
MPC.AX = AX;
MPC.AY = AY;
MPC.DX = DX;
MPC.DY = DY;
MPC.Values = nan([size(AX),length(horizons)]);
MPC.trajectories = cell([size(AX),length(horizons)]);
MPC.data = cell([size(AX),length(horizons)]);
MPC.obstacles = cell([size(AX),length(horizons)]);
MPC.targetData = targetDataOL;

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
    % Print message
    tic
    fprintf('Computing Open Loop solution for [dx,dy] = [%.2f,%.2f]\n',...
            defenderPos(1),defenderPos(2))
    
    %[data, tau, extraOuts] = ...
    % HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
    [dataOL, tauOL, ~] = ...
      HJIPDE_solve(targetDataOL, tau, schemeDataOL, minWith, HJIextraArgsOL);
  
    % Print message
    fprintf('Open Loop solution computed.\t%.2f seconds\n',toc)
      
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
        
        % Print message
        fprintf(['Computing trajectories for initial position:\n',...
                 '[ax,ay,dx,dy] = [%.2f,%.2f,%.2f,%.2f]\n'],...
                 initPos(1),initPos(2),initPos(3),initPos(4));
        
        %% Compute OL optimal trajectory
        % Print message
        tic
        fprintf('Computing Open Loop trajectory...\n')
        
        % find optimal trajectory
        dynSysOL.x = attackerPos; %set initial state

        % [traj, traj_tau] = ...
        % computeOptTraj(g, data, tau, dynSys, extraArgs)
        [trajOL, traj_tauOL] = ...
          computeOptTraj(gOL, dataTrajOL, tauOL, dynSysOL, TrajextraArgs);
        
        % Set up trajectory output
        traj.x = nan(4,length(traj_tauOL));
        traj.value = nan(size(traj_tauOL));
        traj.tau = traj_tauOL;
        
        % Calculate OL defender and value trajectories
        newDefenderPos = defenderPos;
        for iter = 1:length(traj_tauOL)
            dynSysCL.x = [trajOL(:,iter); newDefenderPos]; %set initial state
            traj.x(:,iter) = dynSysCL.x; % store joint state in trajectory
            traj.value(iter) =... % calculate value
               eval_u(gCL,dataCL(clnsCL{:},iter),dynSysCL.x); 
            obsVal  = eval_u(gCL, obstaclesCL, dynSysCL.x); % calc obstacle value
            
            if (iter == length(traj_tauOL)) ||...
               (traj.value(iter) < small) || (obsVal < small)
                break
            end
           
            % Update defender position using CL trajectory
            % [traj, traj_tau] = ...
            % computeOptTraj(g, data, tau, dynSys, extraArgs)
            [trajCL_current, trajCL_tau_current] = ...
                computeOptTraj(gCL, dataTrajCL, tau, dynSysCL,...
                               TrajextraArgsOneStep);    
                           
            if length(trajCL_tau_current) < 2
                a = 1;
            end
                           
            % Update defender position
            newDefenderPos = trajCL_current(3:4,2);  
        end
        
        % Store OL trajectory data in output struct
        OL.trajectories{iax,iay,idx,idy} = traj; 
        
        % Store other OL data in struct
        OL.Values(iax,iay,idx,idy) = traj.value(end);
        OL.data{iax,iay,idx,idy} = dataOL; 
        OL.obstacles{iax,iay,idx,idy} = obstaclesOL;
        
        % Print message
        fprintf('Open Loop trajectory computed.\t%.2f seconds\n',toc)

        %% Compute CL optimal trajectory
        % Print message
        tic
        fprintf('Computing Closed Loop trajectory...\n')
        
        % Store CL value at the beginning
        CL.Values(iax,iay,idx,idy) = ...
            eval_u(gCL,dataCL(clnsCL{:},length(tau)),initPos);

        % find optimal trajectory
        dynSysCL.x = initPos; %set initial state

        % [traj, traj_tau] = ...
         % computeOptTraj(g, data, tau, dynSys, extraArgs)
        [trajCL, traj_tauCL] = ...
          computeOptTraj(gCL, dataTrajCL, tau, dynSysCL, TrajextraArgs);
      
        % Set up trajectory output
        traj.x = trajCL;
        traj.value = nan(size(traj_tauCL));
        traj.tau = traj_tauCL;
      
        % Calculate CL values
        for iter = 1:length(traj_tauCL)
            traj.value(iter) =... % calculate value
               eval_u(gCL,dataCL(clnsCL{:},iter),trajCL(:,iter)); 
        end
        
        % Store CL trajectory data in output struct
        CL.trajectories{iax,iay,idx,idy} = traj;

        % Print message
        fprintf('Closed Loop trajectory computed.\t%.2f seconds\n',toc)
        
        %% Loop through horizons
        for ihz = 1:length(numHorizonSteps)            
          %% Compute MPC optimal trajectory  
          % Print message
          tic
          fprintf('Computing MPC trajectory for horizon of %.2f...\n',horizons(ihz))          
          
          [trajMPC, dataMPC, obsMPC] = ...
            diffGameSolveMPC(agentInfo,numHorizonSteps(ihz),tau,CLin,OLin);
          % diffGameSolveMPC(agentInfo, horizonSteps, tau, CL, OL, mapData)
          
          % Store MPC data in output struct
          MPC.Values(iax,iay,idx,idy,ihz) = trajMPC.value(end);
          MPC.trajectories{iax,iay,idx,idy,ihz} = trajMPC;
          MPC.data{iax,iay,idx,idy,ihz} = dataMPC;
          MPC.obstacles{iax,iay,idx,idy,ihz} = obsMPC;
          
          % Print message
          fprintf(['MPC trajectory computed for horizon of ',...
                   '%.2f.\t%.2f seconds\n'],horizons(ihz),toc)
        end
      end
    end
  end
end
        
%% Visualize
% if visualize
%     figure;
%     plot(trajCL(1,:),trajCL(2,:),'b.-')
%     hold on
%     plot(trajCL(3,:),trajCL(4,:),'c.-')
%     plot(trajOL(1,:),trajOL(2,:),'k.-')
%     plot(trajMPC(1,:),trajMPC(2,:),'r.-')
%     plot(trajMPC(3,:),trajMPC(4,:),'m.-')
%     hold off
% end

end