function r = getRLReward(map, x, dt, captureRadius)
% function to get reward from map, state and time. The defender seeks to
% maximize the reward and the attacker seeks to minimize it.

% Split x into attacker and defender
xa = x(1:2,1);
xd = x(3:4,1);

% Large positive constant (to represent winning/losing)
C = 100;

% Reward for every time step is dt (cost for attacker) 
r = dt;

% Large negative reward if the attacker reaches the target
if isInTarget(map, xa)
    r = r - C;
end

% Large positive reward the defender captures the attacker
if isAttackerCaptured(x, captureRadius)
    r = r + C;
end

% Rewards for obstacles and leaving map (violating position constraints)
% Large positive reward if the attacker is in an obstacle or off of the
% map
if isInObstacle(map, xa) || ~isOnMap(map, xa)
    r = r + C;
end

% Large negative reward if the defender is in an obstacle or off of the
% map
if isInObstacle(map, xd) || ~isOnMap(map, xd)
    r = r - C;
end