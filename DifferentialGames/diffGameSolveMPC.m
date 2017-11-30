function [traj, BRSdata, obsData] = ...
    diffGameSolveMPC(agentInfo, horizonSteps, tau, CL, OL, map)
% Runs solver for MPC approaches for a differential reach-avoid game 
% between two single integrators for time tau and an execution horizon of 
% horizonSteps (number of steps before re-planning). The attacker plans an
% open-loop trajectory after horizonSteps time steps and the defender
% follows an optimal trajectory in response
%
% Inputs:
%   agentInfo - parameters to define the 2 agents (attacker and defender)
%       .initPos       - initial position of attacker and defender in one
%                        vector:    [ax; ay; dx; dy]
%       .captureRadius - minimum distance between agents if attacker is not
%                        captured
%       .dMax          - maximum velocity of the defender
%   horizonSteps - Number of time steps before re-planning open loop
%                  trajectory
%   tau - time vector
%   CL - information from the closed loop solution to this problem.
%            Used to compute value function, defender trajectory, and
%            obstacles
%       .g - grid
%       .data - data for value function
%       .dataTraj - data for value function flipped in time for compute
%                   trajectory function
%       .dynSys - dynSys object for CL system
%       .obstacles - CL obstacle data for checking capture
%       .HJIextraArgs - arguments to pass into HJI solver
%       .TrajextraArgs - arguments to pass into computeTrajectory
%       .clns - cell array of colons
%   OL - information from the open loop solution to this problem. Used
%            to compute backward reachable set (BRS) and obstacles at each
%            time step
%       .g - grid
%       .targetData - data describing target region for HJI solver
%       .dynSys - dynSys object for CL system
%       .schemeData - schemeDat for HJI solver
%       .minWith - minWith input for HJI solver
%       .HJIextraArgs - arguments to pass into HJI solver
%       .TrajextraArgs - arguments to pass into computeTrajectory
%       .clns - cell array of colons
%       .obstacleType - 'ideal' or 'SOS' for circular or square avoid
%                       region
%   mapData - information about the map. E.g. static obstacles
%       .obstacles - static obstacles on the map (in OL state space)
%
% Outputs:
%   traj - struct with state trajectory and value trajectory
%       .x - state trajectory
%       .value - value trajectory
%       .tau - time vector corresponding to trajectory
%   BRSdata - backward reachable set from OL solution at each time step
%   obsData - obstacle data from OL solution at each time step

%% Process input
if nargin < 6
    map = getMap('no_obstacle');
end

if isfield(map,'obstacles')
    staticObstacles = true;
else
    staticObstacles = false;
end

if isfield(OL,'obstacleType')
    obstacleType = OL.obstacleType;
else
    obstacleType = 'ideal';
end
    
%% Setup output variables
% trajectory struct
traj.x = nan(length(agentInfo.initPos),length(tau));
traj.value = nan(size(tau));
traj.tau = tau;

% BRS data
BRSdata = nan([OL.g.shape,length(tau)]);

% obstacle data
obsData = nan([OL.g.shape,length(tau)]);

%% Setup solver arguments
HJIextraArgs = OL.HJIextraArgs;
TrajextraArgs = OL.TrajextraArgs;
TrajextraArgs.trajPoints = 2;

%% Set initial conditions
jointPos = agentInfo.initPos;
attackerPos = jointPos(1:2,1);
defenderPos = jointPos(3:4,1);
traj.x(:,1) = jointPos;

% Stop if value function is within a small tolerance of zero
small = 1e-4;

%% Compute MPC trajectory
for iter = 1:length(tau)
    
    % Find (CL) value of current position at current time
    traj.value(iter) = ...
        eval_u(CL.g, CL.dataTraj(CL.clns{:},iter), jointPos);

    % check if this state is in the BRS/BRT or in an obstacle
    % value = eval_u(g, data, x)
    stopVal = eval_u(CL.g, CL.data(CL.clns{:},1), jointPos);
    obsVal  = eval_u(CL.g, CL.obstacles, jointPos);

    % Stop if in target or obstacle
    if (iter == length(tau)) || (stopVal < small) || (obsVal < small)
        break
    end

    % find optimal trajectory    
    OL.dynSys.x = attackerPos; %set initial state for OL calc
    CL.dynSys.x = jointPos; % set initial state for CL calc

    % If execution horizon has ended, re-compute OL solution
    currentStep = rem(iter-1,horizonSteps) + 1; % current step of the horizon
    if  currentStep == 1 
%         tauCurrent = tau(iter:(iter+horizonSteps-1));
        tauCurrent = tau(iter:end);
                
        % Set up new obstacle for current state
        obsCenter = defenderPos;   
        obstaclesCurrent = ...
           getOpenLoopAvoidSet(OL.g, obsCenter, agentInfo.captureRadius,...
                               agentInfo.dMax,tauCurrent,obstacleType);
       
        % add static obstacles to obstaclesCurrent
        if staticObstacles
            for iObs = 1:length(tauCurrent)
                obstaclesCurrent(OL.clns{:},iObs) = ...
                    min(obstaclesCurrent(OL.clns{:},iObs), map.obstacles);                        
            end
        end
            
        % flip obstacle time points so we start from the beginning of time
        obsTrajCurrent = flip(obstaclesCurrent,ndims(obstaclesCurrent));

        % Set obstacles
        HJIextraArgs.obstacles = obstaclesCurrent;    
        
        % Compute OL value function for current state
        [dataCurrent, tauCurrent, ~] = ...
            HJIPDE_solve(OL.targetData, tauCurrent, OL.schemeData,...
                         OL.minWith, HJIextraArgs);
        
        % flip data time points so we start from the beginning of time
        dataTrajCurrent = flip(dataCurrent,ndims(dataCurrent));
    end
    
    % Store BRS data (forward in time)
    BRSdata(OL.clns{:},iter) = dataTrajCurrent(OL.clns{:},currentStep);
    
    % Store obstacle data (forward in time)
    obsData(OL.clns{:},iter) = obsTrajCurrent(OL.clns{:},currentStep);
    
    % Update defender position using CL trajectory
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [trajCL_current, trajCL_tau_current] = ...
      computeOptTraj(CL.g, CL.dataTraj, tau, CL.dynSys, TrajextraArgs);
  
    % Update defender position
    defenderPos = trajCL_current(3:4,2);    
    
    % Update attacker position using OL trajectory
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [trajOL_current, trajOL_tau_current] = ...
      computeOptTraj(OL.g, dataTrajCurrent, tauCurrent, OL.dynSys, TrajextraArgs);
  
    % Update position
    attackerPos = trajOL_current(:,2);
    
    % Update joint position
    jointPos = [attackerPos; defenderPos];
    
    % Store in trajectory
    traj.x(:,iter+1) = jointPos;
end

% Remove unused positions in trajectory data
traj.x = traj.x(:,1:iter);
traj.value = traj.value(1:iter);
traj.tau = tau(1:iter);

% Remove unused positions in BRSdata and obstacles
BRSdata = BRSdata(OL.clns{:},1:iter);
obsData = obsData(OL.clns{:},1:iter);

% flip BRS data time points so we start from end of time
BRSdata = flip(BRSdata,ndims(BRSdata));

% flip obstacle time points so we start from the end of time
obsData = flip(obsData,ndims(obsData));
