function [f,v,traj] = simulateRL(x0,map, w, tau, dt,captureRadius, dynSys, options, f)

step = 1;

visualize = true;
if isfield(options,'visualize')
    visualize = options.visualize;        
end

if visualize
    if nargin < 8
        f = plotMap(map);
    else
        f = plotMap(map,f);
    end
end



% Initialize random state in state space
stateMin = repmat(map.grid_min, 2, 1);
stateRange = repmat(map.grid_max - map.grid_min, 2, 1);

dynSys.x = x0;
% dynSys.x = rand(4,1).*stateRange + stateMin;

% dynSys.x = [-2; 1; -2; 3];

if visualize
    % define capture circle
    thetas = linspace(0,2*pi,1000); 
    circlePoints = captureRadius*[cos(thetas); sin(thetas)]; 




    axisVector = ...
        [stateMin(1)-captureRadius,stateMin(1)+stateRange(1)+captureRadius,...
         stateMin(2)-captureRadius,stateMin(2)+stateRange(2)+captureRadius];

    clf
    hold on
    plot(dynSys.x(1),dynSys.x(2),'b.','MarkerSize',10);
    plot(dynSys.x(3),dynSys.x(4),'r^','MarkerFaceColor','r');
    plot(circlePoints(1,:)+dynSys.x(3),circlePoints(2,:)++dynSys.x(4),'k')
    plotMap(map,f);
    hold off
    axis(axisVector);
    axis square
    drawnow
end

traj = nan(4,length(tau));
traj(:,1) = dynSys.x;
while step < length(tau) && ~isDiffGameEnd(map,dynSys.x,captureRadius)
    u = getRLAction(dynSys.x, w, dt, dynSys, 'min',options);
    d = getRLAction(dynSys.x, w, dt, dynSys, 'max',options);
    
    % Update state with these actions
    dynSys.updateState(u, dt, dynSys.x, d);
    
    if visualize
        clf
        plotMap(map,f);
        hold on
        plot(dynSys.x(1),dynSys.x(2),'b.','MarkerSize',10);
        plot(dynSys.x(3),dynSys.x(4),'r^','MarkerFaceColor','r');
        plot(circlePoints(1,:)+dynSys.x(3),circlePoints(2,:)++dynSys.x(4),'k')
        hold off
        axis(axisVector);
        axis square
        drawnow    
    end
    
    step = step + 1; 
    traj(:,step) = dynSys.x;
end
traj = traj(:,1:step);


xa = dynSys.x(1:2,1);
if isInTarget(map, xa)
    v = -1;
else
    v = 1;
end