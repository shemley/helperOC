function f = simulateRL(map, w, tau,dt,captureRadius, dynSys, f)

step = 1;

% Initialize random state in state space
stateMin = repmat(map.min, 2, 1);
stateRange = repmat(map.max - map.min, 2, 1);

dynSys.x = rand(4,1).*stateRange + stateMin;

if nargin < 7
    f = plotMap(map);
else
    f = plotMap(map,f);
end

hold on
plot(dynSys.x(1),dynSys.x(2),'b.','MarkerSize',10);
plot(dynSys.x(3),dynSys.x(4),'r^','MarkerFaceColor','r');
hold off
drawnow

while step <= length(tau) && ~isDiffGameEnd(map,dynSys.x,captureRadius)
    u = getRLAction(dynSys.x, w, dynSys, 'min');
    d = getRLAction(dynSys.x, w, dynSys, 'max');
    
    % Update state with these actions
    dynSys.updateState(u, dt, dynSys.x, d);
    
    plotMap(map,f);
    hold on
    plot(dynSys.x(1),dynSys.x(2),'b.','MarkerSize',10);
    plot(dynSys.x(3),dynSys.x(4),'r^','MarkerFaceColor','r');
    hold off
    drawnow    
    
    step = step + 1; 
end