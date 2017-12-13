function [OL,CL,MPC]= diffGameCompare(icGrid, agentInfo, horizons, tMax, map, extraArgs)
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
%   map - struct retrieved with the getMap function containing info on
%         static aspects of the game (grid, target, obstacles)
%       .name
%       .min
%       .max
%       .N
%       .g
%       .target
%           .radius
%           .center
%           .data
%       .obstacles - might not be present
%   extraArgs - struct to store any extra arguments
%       .visualize - set to true to visualize results (defaults to false)
%       .computeTraj - set to true to compute CL and OL trajectories
%                      (defaults to true). MPC trajectory is required
%       .obstacleType - 'ideal' uses circles for OL and MPC capture avoid
%                       sets. 'SOS' uses conservative squares to reflect 
%                       the SOS method's conservatism
%       .dMaxError - percent error in defender dynamics model (from
%                    attacker's perspective). dMaxError > 0 means attacker
%                    underestimates dMax. dMaxError < 0 means attacker
%                    overestimates dMax
%       .savedCL - saved CL solution that can be used to reduce computation
%                  time       .
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

% Map input
if nargin < 5
    map = getMap('no_obstacle');
end

% Check for static obstacles
staticObstacles = isfield(map,'obstacles');

% If no extra args, use defaults
visualize = false; % don't visualize results
computeTraj = true; % compute CL and OL trajectories
obstacleType = 'ideal'; % use ideal obstacles
modelError = false; % don't include modeling error for dMax
dMaxError = 0; % default to zero modeling error
computeCL = true; % compute new CL solution
saveCLresult = true; % save new computed CL solution

if nargin >= 6
    % Check for visualization input
    if isfield(extraArgs,'visualize')
        visualize = extraArgs.visualize;
    end
    
    % Check for compute trajectory input
    if isfield(extraArgs,'computeTraj')
        computeTraj = extraArgs.computeTraj;
    end
    
    % Check for obstacle type input
    if isfield(extraArgs,'obstacleType')
        obstacleType = extraArgs.obstacleType;
    end
    
    % Check for dMax error input
    if isfield(extraArgs,'dMaxError')
        if abs(extraArgs.dMaxError) > 1e-4 % consider only non-zero error 
            modelError = true;
            dMaxError = extraArgs.dMaxError;
        end
    end
    
    % Check for saved CL solution input
    if isfield(extraArgs,'savedCL')
        % Use input data provided that it is for the same parameters
        computeCL = false;
        saveCLresult = false;

        % Check that the parameters are provided along with the input CL data
        if isfield(extraArgs.savedCL,'uMax') && ...
           isfield(extraArgs.savedCL,'dMax') && ...
           isfield(extraArgs.savedCL,'captureRadius') &&...
           isfield(extraArgs.savedCL,'tMax') &&...
           isfield(extraArgs.savedCL,'mapName')

            % Check that CL data matches the input parameters
            if extraArgs.savedCL.uMax == uMax && ...
               extraArgs.savedCL.dMax == dMax &&...
               extraArgs.savedCL.captureRadius == captureRadius && ...
               extraArgs.savedCL.tMax == tMax && ...
               strcmp(extraArgs.savedCL.mapName,map.name)

                % Use saved data
                dataCL = extraArgs.savedCL.data;
            else
               error('Saved CL data doesn''t match input parameters.')
            end
        else
          error('Saved CL data must be in a struct with corresponding parameters.')
        end
    end    
end


%% General parameters (OL and CL)

% % Grid parameters
grid_min = map.grid_min; % Lower corner of computation domain
grid_max = map.grid_max;   % Upper corner of computation domain
N = map.N;        % Number of grid points per dimension
pdDims  = [];        % no periodic dimensions. 

% To be deleted. Defined in map instead
% % Target set
% targetRadius = 1;
% targetCenter = [0; 0];

% Calculate dMax with Error
dMaxWithError = (1+dMaxError)*dMax;

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
% record CL computation time
tic

% Colons for taking time slices of data (e.g. data(clns,time))
clnsCL = repmat({':'}, 1, 4);

% Define dynamic system
% obj = KinVehicleND2Agent(x, uMax, dMax)
dynSysCL = KinVehicleND2Agent([0,0,0,0], uMax, dMax);
pdDimsCL = [];        % no periodic dimensions. 

% Create CL grid using grid parameters defined above concatenated into 4D
% for attacker and defender positions
gCL = createGrid([grid_min; grid_min],[grid_max; grid_max],[N; N],pdDimsCL);

% Target set (concatenate into 4D
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
ignoreDims = [3,4]; % target is only in attacker dimensions
% target set for attacker is in map structure, ignore defender
targetDataCL = nan(gCL.shape);%shapeCylinder(gCL,ignoreDims,...
     %            [map.target.center; map.target.center],map.target.radius); 
% populate CL target data from OL map data
% alternatively could use:
% targetDataCL = 
% repmat(map.obstacles,1,1,size(targetDataCL,3),size(targetDataCL,4));
for i3 = 1:size(targetDataCL,3)
    for i4 = 1:size(targetDataCL,4)
        targetDataCL(:,:,i3,i4) = map.target.data;
    end
end
     

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

% Include static obstacles from map
if staticObstacles
    attackerObs = nan(size(targetDataCL));
    defenderObs = nan(size(targetDataCL));
    
    % Set static obstacles for attacker
    % alternatively could use:
    % attackerObs =
    % repmat(map.obstacles,1,1,size(attackerObs,3),size(attackerObs,4));
    for i3 = 1:size(attackerObs,3)
        for i4 = 1:size(attackerObs,4)
            attackerObs(:,:,i3,i4) = map.obstacles;
        end
    end
    
    % Set static obstacles for defender
    % alternatively could use:
    % defenderObs =
    % repmat(map.obstacles,size(defenderObs,1),size(defenderObs,2),1,1);
    for i1 = 1:size(defenderObs,1)
        for i2 = 1:size(defenderObs,2)
            defenderObs(i1,i2,:,:) = map.obstacles;
        end
    end
    
    obstaclesCL = min(obstaclesCL, attackerObs);
    targetDataCL = min(targetDataCL, defenderObs); % add defender static obstacles to target data
end

% Set obstacles
HJIextraArgsCL.obstacles = obstaclesCL;

% Parameters for CL with error
if modelError
    % DynSys
    dynSysCLWithError = KinVehicleND2Agent([0,0,0,0], uMax, dMaxWithError);
    % scheme data
    schemeDataCLWithError = schemeDataCL;
    schemeDataCLWithError.dynSys = dynSysCLWithError;
end

% record comp time
compTimeCL = toc;

%% Open Loop (OL) parameters
% record OL comp time
tic

% Colons for taking time slices of data (e.g. data(clns,time))
clnsOL = repmat({':'}, 1, 2);

% Define dynamic system
% obj = KinVehicleND(x, uMax)
dynSysOL = KinVehicleND([0,0], uMax);
pdDimsOL = [];

% Create OL grid using grid parameters defined above
gOL = createGrid(grid_min, grid_max, N, pdDimsOL);

% data0 = shapeCylinder(grid,ignoreDims,center,radius)
targetCenter = map.target.center;
% target set for attacker is a circle (given in map)
targetDataOL = map.target.data; %shapeSphere(gOL, targetCenter, map.target.radius); 

% Put grid and dynamic systems into schemeData
schemeDataOL.grid = gOL;
schemeDataOL.dynSys = dynSysOL;
schemeDataOL.accuracy = 'high'; %set accuracy
schemeDataOL.uMode = uMode;

% Solver extra parameters
HJIextraArgsOL.visualize = false; %show plot

% Record OL comp time
compTimeOL = toc;

%% Compute CL value function
% Record CL comp time
tic
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
compTimeCL = compTimeCL + toc;

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
    savedCL.mapName = map.name;
    
    clFilename = sprintf(...
       './DifferentialGames/CLresults_%s_u%.1f_d%.1f_cr%.1f_t%.1f_n%d.mat',...
                                   map.name,uMax,dMax,captureRadius,tMax,N(1));
    save(clFilename,'savedCL')
    
    % Print message
    fprintf('Closed Loop solution saved.\t%.2f seconds\n',toc)
end

% record cl comp time
tic

% For trajectory computation:
%flip data time points so we start from the beginning of time
dataTrajCL = flip(dataCL,ndims(dataCL));
compTimeCL = compTimeCL + toc;

% If modeling error, compute value function with error
if modelError
    % Print message
    tic
    fprintf('Computing Closed Loop solution with error...\n')
    
    % %[data, tau, extraOuts] = ...
    % % HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
    [dataCLWithError, tauCLWithError, ~] = ...
      HJIPDE_solve(targetDataCL, tau, schemeDataCLWithError, minWith, HJIextraArgsCL);
  
    % data with Error traj
    dataTrajCLWithError = flip(dataCLWithError,ndims(dataCLWithError));
  
    % Print message
    fprintf('Closed Loop solution with error computed.\t%.2f seconds\n',toc)
end

%% Set up input for MPC solver
% Agent info
agentInfoMPC = agentInfo;
agentInfoMPC.dMax = dMaxWithError;

% OL parameter structure
OLin.g = gOL;
OLin.targetData = targetDataOL;
OLin.dynSys = dynSysOL;
OLin.schemeData = schemeDataOL;
OLin.minWith = minWith;
OLin.HJIextraArgs = HJIextraArgsOL;
OLin.TrajextraArgs = TrajextraArgs;
OLin.clns = clnsOL;
OLin.obstacleType = obstacleType;

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
CL.g = gCL;
CL.dMax = dMax;
CL.dMaxError = dMaxError;
CL.dMaxWithError = dMaxWithError;
CL.Values = nan(size(AX));
CL.trajectories = cell(size(AX));
CL.data = dataCL;
CL.obstacles = obstaclesCL;
CL.targetData = targetDataCL;
CL.TrajextraArgs = TrajextraArgs;

if modelError
    CL.dataWithError = dataCLWithError;
end

% Open loop
OL.AX = AX;
OL.AY = AY;
OL.DX = DX;
OL.DY = DY;
OL.g = gOL;
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
MPC.g = gOL;
MPC.Values = nan([size(AX),length(horizons)]);
MPC.trajectories = cell([size(AX),length(horizons)]);
MPC.data = cell([size(AX),length(horizons)]);
MPC.obstacles = cell([size(AX),length(horizons)]);
MPC.targetData = targetDataOL;
MPC.compTime = cell(size(horizons));

%% Loop through Defender positions
for idx = 1:size(DX,3)
  for idy = 1:size(DY,4)
    % Set defender position
    defenderPos = [DX(1,1,idx,idy); DY(1,1,idx,idy)]; 
      
    %% Compute OL obstacles
    % record OL comp time
    tic
    
    % Define Obstacles. Use dMaxWithError to consider cases with zero and
    % non-zero error
    obsCenter  = defenderPos;   
    obstaclesOL = getOpenLoopAvoidSet(gOL,obsCenter,captureRadius,dMaxWithError,tau,obstacleType);
        
    if staticObstacles
        for iObs = 1:length(tau)
            obstaclesOL(clnsOL{:},iObs) = ...
                min(obstaclesOL(clnsOL{:},iObs), map.obstacles);
        end
    end
    
    % Set obstacles
    HJIextraArgsOL.obstacles = obstaclesOL;
    compTimeOL = compTimeOL + toc;
    
    
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
    
    % record ol comp time
    compTimeOL = compTimeOL + toc;

    %% Loop through attacker positions
    for iax = 1:size(AX,1)
      for iay = 1:size(AY,2)
        % Set attacker position  
        attackerPos = [AX(iax,iay,idx,idy); AY(iax,iay,idx,idy)];  
        % Set joint inital position
        initPos = [attackerPos; defenderPos];
        % Set agentInfo for MPC solver
        agentInfoMPC.initPos = initPos;
        
        % Print message
        fprintf(['Computing trajectories for initial position:\n',...
                 '[ax,ay,dx,dy] = [%.2f,%.2f,%.2f,%.2f]\n'],...
                 initPos(1),initPos(2),initPos(3),initPos(4));
        
        %% Compute OL optimal trajectory
        if computeTraj        
            % Print message
            tic
            fprintf('Computing Open Loop trajectory...\n')

            % find optimal trajectory
            dynSysOL.x = attackerPos; %set initial state

            % [traj, traj_tau] = ...
            % computeOptTraj(g, data, tau, dynSys, extraArgs)
            [trajOL, traj_tauOL] = ...
              computeOptTraj(gOL, dataTrajOL, tauOL, dynSysOL, TrajextraArgs);

            % Record comp time
            compTimeOL = compTimeOL + toc;
          
            % Set up trajectory output
            traj.x = nan(4,length(traj_tauOL));
            traj.value = nan(size(traj_tauOL));
            traj.tau = traj_tauOL;
            tEarliest = 1;

            % Calculate OL defender and value trajectories
            newDefenderPos = defenderPos;
            for iter = 1:length(traj_tauOL)
                dynSysCL.x = [trajOL(:,iter); newDefenderPos]; %set initial state
                
                % Determine the earliest time that the current state is in the reachable set
                % Binary search
                upper = length(tau);
                lower = 1;
  
                tEarliest = find_earliest_BRS_ind(gCL, dataTrajCL, dynSysCL.x, upper, lower);
                
                traj.x(:,iter) = dynSysCL.x; % store joint state in trajectory
                traj.value(iter) =... % calculate value
                   eval_u(gCL,dataTrajCL(clnsCL{:},tEarliest),dynSysCL.x);
                stopVal = eval_u(gCL, dataCL(clnsCL{:},1), dynSysCL.x); % calc target val
                obsVal  = eval_u(gCL, obstaclesCL, dynSysCL.x); % calc obstacle value

                if (iter == length(traj_tauOL)) || (stopVal < small) || (obsVal < small)
                    break
                end

                % Update defender position using CL trajectory
                % [traj, traj_tau] = ...
                % computeOptTraj(g, data, tau, dynSys, extraArgs)
                [trajCL_current, trajCL_tau_current] = ...
                    computeOptTraj(gCL, dataTrajCL, tau, dynSysCL,...
                                   TrajextraArgsOneStep);    

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

            if ~modelError
                % [traj, traj_tau] = ...
                 % computeOptTraj(g, data, tau, dynSys, extraArgs)
                [trajCL, traj_tauCL] = ...
                  computeOptTraj(gCL, dataTrajCL, tau, dynSysCL, TrajextraArgs);
                
                traj.x = trajCL;
                traj.tau = traj_tauCL;
                traj.value = nan(size(traj_tauCL));                
              
                % Calculate CL values
                for iter = 1:length(traj_tauCL)
                    % Determine the earliest time that the current state is in the reachable set
                    % Binary search
                    upper = length(tau);
                    lower = 1;
  
                    tEarliest = find_earliest_BRS_ind(gCL, dataTrajCL, trajCL(:,iter), upper, lower);
                    
                    traj.value(iter) =... % calculate value
                        eval_u(gCL,dataTrajCL(clnsCL{:},tEarliest),trajCL(:,iter)); 
                end
                
                compTimeCL = compTimeCL + toc;
            else                
                traj.x = nan(length(initPos),length(tau));
                traj.value = nan(size(tau));
                traj.tau = tau;
                
                jointPos = initPos;
                traj.x(:,1) = initPos;
                
                for iter = 1:length(tau)   
                    % Determine the earliest time that the current state is in the reachable set
                    % Binary search
                    upper = length(tau);
                    lower = 1;
  
                    tEarliest = find_earliest_BRS_ind(gCL, dataTrajCL, jointPos, upper, lower);
                    
                    % Find (CL) value of current position at current time
                    traj.value(iter) = ...
                        eval_u(gCL, dataTrajCL(clnsCL{:},tEarliest), jointPos);

                    % check if this state is in the BRS/BRT or in an obstacle
                    % value = eval_u(g, data, x)
                    stopVal = eval_u(gCL, dataCL(clnsCL{:},1), jointPos);
                    obsVal  = eval_u(gCL, obstaclesCL, jointPos);

                    % Stop if in target or obstacle
                    if (iter == length(tau)) || (stopVal < small) || (obsVal < small)
                        break
                    end

                    % find optimal trajectory    
                    dynSysCL.x = jointPos; %set initial state for actual defender traj calc
                    dynSysCLWithError.x = jointPos; % set initial state for attacker calc (with error)

                    % Update defender position using CL without error
                    % [traj, traj_tau] = ...
                    % computeOptTraj(g, data, tau, dynSys, extraArgs)
                    [trajCL_def, trajCL_tau_def] = ...
                      computeOptTraj(gCL, dataTrajCL, tau, dynSysCL, TrajextraArgsOneStep);

                    % Update defender position
                    newDefenderPos = trajCL_def(3:4,2);    

                    % Update attacker position using CL with error
                    % [traj, traj_tau] = ...
                    % computeOptTraj(g, data, tau, dynSys, extraArgs)
                    [trajCL_att, trajCL_tau_att] = ...
                      computeOptTraj(gCL, dataTrajCLWithError, tau, dynSysCLWithError, TrajextraArgsOneStep);

                    % Update position
                    newAttackerPos = trajCL_att(1:2,2);

                    % Update joint position
                    jointPos = [newAttackerPos; newDefenderPos];

                    % Store in trajectory
                    traj.x(:,iter+1) = jointPos;
                end
                
                % Set up trajectory output
                traj.x = traj.x(:,1:iter);
                traj.value = traj.value(1:iter);
                traj.tau = tau(1:iter);               
            end

            % Store CL trajectory data in output struct
            CL.trajectories{iax,iay,idx,idy} = traj;

            % Print message
            fprintf('Closed Loop trajectory computed.\t%.2f seconds\n',toc)        
        end
        %% Loop through horizons
        for ihz = 1:length(numHorizonSteps)            
          %% Compute MPC optimal trajectory  
          % Print message
          tic
          fprintf('Computing MPC trajectory for horizon of %.2f...\n',horizons(ihz))          
          
          [trajMPC, dataMPC, obsMPC, compTimeMPC] = ...
            diffGameSolveMPC(agentInfoMPC,numHorizonSteps(ihz),tau,CLin,OLin,map);
        
          % Store MPC data in output struct
          MPC.Values(iax,iay,idx,idy,ihz) = trajMPC.value(end);
          MPC.trajectories{iax,iay,idx,idy,ihz} = trajMPC;
          MPC.data{iax,iay,idx,idy,ihz} = dataMPC;
          MPC.obstacles{iax,iay,idx,idy,ihz} = obsMPC;
          MPC.compTime{ihz} = compTimeMPC;
          
          % Print message
          fprintf(['MPC trajectory computed for horizon of ',...
                   '%.2f.\t%.2f seconds\n'],horizons(ihz),toc)
        end
      end
    end
  end
end
        
CL.compTime = compTimeCL;
OL.compTime = compTimeOL;


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