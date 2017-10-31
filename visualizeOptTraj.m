%% visualize optimal trajectory from data
function visualizeOptTraj(g, data, traj, tau, extraArgs)

% Check extra args input
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

% Grid params
clns = repmat({':'}, 1, g.dim);

% Time parameters
iter = 1;
tauLength = length(tau);
tEarliest = 1;


while iter <= tauLength
    % Determine the earliest time that the current state is in the reachable set
    % Binary search
    upper = tauLength;
    lower = tEarliest;
    
    tEarliest = find_earliest_BRS_ind(g, data, traj(:,iter), upper, lower);
    
    % BRS at current time
    BRS_at_t = data(clns{:}, tEarliest);
    
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
    
    hold off        
    
    if isfield(extraArgs, 'fig_filename')
        export_fig(sprintf('%s%d', extraArgs.fig_filename, iter), '-png')
    end
    
    % Increment iterator
    iter = iter + 1;
end
