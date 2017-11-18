function obstacles = getOpenLoopAvoidSet(g, center, radius, dMax, tau, type)
% function to compute open loop obstacles for differential reach avoid
% games. This amount to a time varying obstacle starting as a circle of
% radius centered at center that expands by dMax per unit of time. tau is
% the time vector for the computation

if nargin < 6
    type = 'ideal';
end

clns = repmat({':'}, 1, g.dim);

if strcmp(type,'ideal')
    % open loop obstacle is circle in the attacker dimensions
    % (1 and 2) centered at defender initial position
    obsfunc = @(i) shapeSphere(g, center, radius + dMax*(tau(i)-tau(1)));
elseif strcmp(type,'SOS')
    % use rectangle to simulate SOS solution
    obsfunc = @(i) shapeRectangleByCenter(g, center, repmat(2*(radius + dMax*(tau(i)-tau(1))), 1, g.dim));
else
    error('Invalid obstacle type. Must be ideal or SOS')
end

obstacles = zeros([g.N',length(tau)]);  
for i = 1:length(tau)
    % Call obstacle function defined above
    obstacles(clns{:},i) = obsfunc(i);
    
%     % open loop obstacle is circle in the attacker dimensions
%     % (1 and 2) centered at defender initial position
%     obstacles(clns{:},i) =...
%         shapeSphere(g, center, radius + dMax*(tau(i)-tau(1)));
%        %shapeRectangleByCenter(g, obsCenter, [captureRadius + dMax*tau(i), captureRadius + dMax*tau(i)]);  % use rectangle to simulate SOS solution
end

% Flip obstacle array to start at the end of time for BRT solver
obstacles = flip(obstacles,ndims(obstacles));