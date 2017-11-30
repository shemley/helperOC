function actions = getRLLegalActions(vMax)
% Returns all discretized actions for RL


% Discretize action space.
numActions = 16;
thetas = linspace(0,2*pi,numActions+1);
thetas = thetas(1:(end-1));

actions = vMax.*[cos(thetas); sin(thetas)];
actions = [[0;0] , actions];