function isEnd = isDiffGameEnd(map, x, captureRadius)
% Check for the end of a differential game

% split states into attacker and defender
xa = x(1:2,1);
xd = x(3:4,1);

% Game over if the attacker has reached the target, if the defender has
% caught the attacker, or if either has violated their constraints (entered
% an obstacle or left the map).
isEnd = (isInTarget(map, xa) || isAttackerCaptured(x, captureRadius) || ... % target or capture
         isInObstacle(map, xa) || isInObstacle(map, xd) || ... % obstacles
         ~isOnMap(map, xa) || ~isOnMap(map,xd)); % either player off the map