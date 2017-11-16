function [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
% [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
%   Computes the optimal trajectories given the optimal value function
%   represented by (g, data), associated time stamps tau, dynamics given in
%   dynSys.
%
% Inputs:
%   g, data - grid and value function
%   tau     - time stamp (must be the same length as size of last dimension of
%                         data)
%   dynSys  - dynamical system object for which the optimal path is to be
%             computed
%   extraArgs
%     .uMode        - specifies whether the control u aims to minimize or
%                     maximize the value function
%     .trajPoints   - specifies the number of points to compute along the
%                     trajectory (defaults to length(tau))
%     .visualize    - set to true to visualize results
%     .fig_num:   List if you want to plot on a specific figure number
%     .projDim      - set the dimensions that should be projected away when
%                     visualizing
%     .distDim      - set the dimensions of the disturbance to project to
%                     the same plot. Used to visualize an adversary
%     .targetData   - data to plot target region
%     .targetCenter - center of target region
%     .obstacleData - data to plot obstacles
%     .fig_filename - specifies the file name for saving the visualizations

if nargin < 5
  extraArgs = [];
end

% Default parameters
uMode = 'min';
dMode = 'max';
trajPoints = length(tau);
visualize = false;
subSamples = 4;

if isfield(extraArgs, 'uMode')
  uMode = extraArgs.uMode;
end

if isfield(extraArgs, 'dMode')
  dMode = extraArgs.dMode;
end

if isfield(extraArgs, 'trajPoints')
  trajPoints = extraArgs.trajPoints;
end

% Visualization
if isfield(extraArgs, 'visualize') && extraArgs.visualize
  visualize = extraArgs.visualize;
  
  showDims = find(extraArgs.projDim);
  hideDims = ~extraArgs.projDim;
  
  if isfield(extraArgs,'distDim')
    distShowDims = find(extraArgs.distDim);
    distHideDims = ~extraArgs.distDim;
  end
  
  if isfield(extraArgs,'targetData')
    targetData = extraArgs.targetData;
  end
  
  if isfield(extraArgs,'targetCenter')
    targetCenter = extraArgs.targetCenter;
  else
    targetCenter = zeros(size(projDim));
  end
  
  if isfield(extraArgs,'obstacleData')
    obstacleData = extraArgs.obstacleData;
    
    if length(size(obstacleData)) == g.dim
        timeVaryingObs = false;
    else
        timeVaryingObs = true;
    end
  end
  
  if isfield(extraArgs,'fig_num')
    f = figure(extraArgs.fig_num);
  else
    f = figure;
  end
end

if isfield(extraArgs, 'subSamples')
  subSamples = extraArgs.subSamples;
end

clns = repmat({':'}, 1, g.dim);

if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

% Time parameters
iter = 1;
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/subSamples;
% maxIter = 1.25*tauLength;

% Initialize trajectory
traj = nan(g.dim, tauLength);
traj(:,1) = dynSys.x;
tEarliest = 1;

while iter <= trajPoints 
  % Determine the earliest time that the current state is in the reachable set
  % Binary search
  upper = tauLength;
  lower = tEarliest;
  
  tEarliest = find_earliest_BRS_ind(g, data, dynSys.x, upper, lower);
   
  % BRS at current time
  BRS_at_t = data(clns{:}, tEarliest);
  
  % Visualize BRS corresponding to current trajectory point
  if visualize
    plot(traj(showDims(1), iter), traj(showDims(2), iter), 'b.','MarkerSize',15)
    hold on
    [g2D, data2D] = proj(g, BRS_at_t, hideDims, traj(hideDims,iter));
    visSetIm(g2D, data2D,'b');
    
    % Show adversary (disturbance) if one exists and capture set
    if exist('distShowDims','var')
        plot(traj(distShowDims(1), iter), traj(distShowDims(2), iter), 'r^','MarkerFaceColor','r')
        [distG2D, distData2D] = proj(g, BRS_at_t, distHideDims, traj(distHideDims,iter));
        visSetIm(distG2D, distData2D,'r');
        
        % plot obstacle
        if exist('obstacleData','var')
            if timeVaryingObs
                obs2use = obstacleData(clns{:},iter);
            else
                obs2use = obstacleData;
            end
            
            [obsG2D, obsData2D] = proj(g, obs2use, hideDims, traj(hideDims,iter));
            visSetIm(obsG2D, obsData2D,'k');
        end
    end
    
    % Show target if it exists (in agent's coordinates
    if exist('targetData','var')
        [targetG2D, targetData2D] = proj(g, targetData, hideDims, targetCenter(hideDims));
        visSetIm(targetG2D, targetData2D,'g');
    end
    
    tStr = sprintf('t = %.3f; tEarliest = %.3f', tau(iter), tau(tEarliest));
    title(tStr)
    drawnow
    
    if isfield(extraArgs, 'fig_filename')
      export_fig(sprintf('%s%d', extraArgs.fig_filename, iter), '-png')
    end

    hold off
  end
  
  % If this is the last point, break the loop to avoid re-computing the
  % gradients and increment the iterator to include the current point
  if (iter == trajPoints)
    iter = iter + 1;
    break
  end
      
  if (tEarliest == tauLength)
    % Trajectory has entered the target
    break
  end
  
  % Update trajectory
  Deriv = computeGradients(g, BRS_at_t);
  for j = 1:subSamples
    deriv = eval_u(g, Deriv, dynSys.x);
    u = dynSys.optCtrl(tau(tEarliest), dynSys.x, deriv, uMode);
    d = dynSys.optDstb(tau(tEarliest), dynSys.x, deriv, dMode); % add disturbance
    dynSys.updateState(u, dtSmall, dynSys.x, d);
  end
  
  % Record new point on nominal trajectory
  iter = iter + 1;
  traj(:,iter) = dynSys.x;
end

% Delete unused indices
traj(:,iter:end) = [];
traj_tau = tau(1:iter-1);
end