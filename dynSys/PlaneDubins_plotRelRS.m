function h = PlaneDubins_plotRelRS(evader, pursuer, g, data, extraArgs)
% h = PlaneDubins_plotRelRS(evader, pursuer, g, data, extraArgs)
%     Plots a relative reachable set (eg. max min or min min set between two
%     vehicles) between a Plane/DubinsCar and a Plane/DubinsCar
%
% Inputs:
%     evader    - the vehicle centered at the origin in relative coordinates
%     pursuer   - the other vehicle
%     g, data   - grid and value function on the grid
%     extraArgs - plotting parameters; see visSetIm
% 
% Output:
%     h - handle of plotted object

if nargin < 2
  extraArgs.LineStyle = '--';
  extraArgs.LineWidth = 2;
end

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

[g2D, data2D] = proj(g, data, [0 0 1], x_rel(3));
dataRot = rotateData(g2D, data2D, evader.x(3), [1 2], []);
gShift = shiftGrid(g2D, evader.x(1:2));

h = visSetIm(gShift, dataRot, 'r', 0, extraArgs);
end