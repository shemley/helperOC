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
%     .visualize    - set to true to visualize results
%     .projDim      - set the dimensions that should be projected away when
%                     visualizing
%     .fig_filename - specifies the file name for saving the visualizations

if nargin < 5
  extraArgs = [];
end

% Default parameters
uMode = 'min';
visualize = false;
subSamples = 4;

if isfield(extraArgs, 'uMode')
  uMode = extraArgs.uMode;
end

% Visualization
if isfield(extraArgs, 'visualize') && extraArgs.visualize
  visualize = extraArgs.visualize;
  
  showDims = find(extraArgs.projDim);
  hideDims = ~extraArgs.projDim;
  
  figure
end

if isfield(extraArgs, 'subSamples')
  subSamples = extraArgs.subSamples;
end

clns = repmat({':'}, 1, g.dim);

if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

% Time parameters
small = 1e-4;
BRS_t = 1;
traj_t = 1;
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/subSamples;

% Initialize trajectory
traj = nan(3, tauLength);
traj(:,1) = dynSys.x;

while BRS_t <= tauLength
  % Determine the earliest time that the current state is in the reachable set
  for tEarliest = tauLength:-1:BRS_t
    valueAtX = eval_u(g, data(clns{:}, tEarliest), dynSys.x);
    if valueAtX < small
      break
    end
  end
  
  % BRS at current time
  BRS_at_t = data(clns{:},tEarliest);
  
  % Visualize BRS corresponding to current trajectory point
  if visualize
    plot(traj(showDims(1), traj_t), traj(showDims(2), traj_t), 'k.')
    hold on
    [g2D, data2D] = proj(g, BRS_at_t, hideDims, traj(hideDims,traj_t));
    visSetIm(g2D, data2D);
    tStr = sprintf('t = %.3f; tEarliest = %.3f', tau(traj_t), tau(tEarliest));
    title(tStr)
    drawnow
    
    if isfield(extraArgs, 'fig_filename')
      export_fig(sprintf('%s%d', extraArgs.fig_filename, traj_t), '-png')
    end

    hold off
  end
  
  if tEarliest == tauLength
    % Trajectory has entered the target
    break
  end
  
  % Update trajectory
  Deriv = computeGradients(g, BRS_at_t);
  for j = 1:subSamples
    deriv = eval_u(g, Deriv, dynSys.x);
    u = dynSys.optCtrl(tau(BRS_t), dynSys.x, deriv, uMode);
    dynSys.updateState(u, dtSmall, dynSys.x);
  end
  
  % Record new point on nominal trajectory
  traj_t = traj_t + 1;
  traj(:,traj_t) = dynSys.x;
  BRS_t = tEarliest + 1;
end

% Delete unused indices
traj(:,traj_t:end) = [];
traj_tau = tau(1:traj_t-1);
end