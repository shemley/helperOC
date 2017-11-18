function phi = getRLFeatures(x)
% function to return feature vector for RL given state

% Initialize 6 features
phi    = zeros(6,1);

% Features in states
phi(1) = x(1);
phi(2) = x(2);
phi(3) = x(3);
phi(4) = x(4);

% Distance features
phi(5) = norm(x(1:2) - x(3:4),2); % Distance between agents
phi(6) = norm(x(1:2)); % Distance of attacker to target

phi = 1./(1+exp(-phi));