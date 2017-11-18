function isCaptured = isAttackerCaptured(x, captureRadius)
% check if defender has captured attacker

% split x into attacker and defender
xa = x(1:2,1);
xd = x(3:4,1);

% Check for capture
isCaptured = norm(xa - xd, 2) <= captureRadius;