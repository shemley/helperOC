function obstacles = getOpenLoopAvoidSet(g, center, radius, dMax, tau)
% function to compute open loop obstacles for differential reach avoid
% games. This amount to a time varying obstacle starting as a circle of
% radius centered at center that expands by dMax per unit of time. tau is
% the time vector for the computation

clns = repmat({':'}, 1, g.dim);

obstacles = zeros([g.N',length(tau)]);  
for i = 1:length(tau)
    % open loop obstacle is circle in the attacker dimensions
    % (1 and 2) centered at defender initial position
    obstacles(clns{:},i) =...
        shapeSphere(g, center, radius + dMax*(tau(i)-tau(1)));
       %shapeRectangleByCenter(g, obsCenter, [captureRadius + dMax*tau(i), captureRadius + dMax*tau(i), Inf, Inf]);  % could use rectangle to simulate SOS solution
end

% Flip obstacle array to start at the end of time for BRT solver
obstacles = flip(obstacles,ndims(obstacles));