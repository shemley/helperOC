function h = PlaneDubins_plotAbsRS(vehicle, g, data, extraArgs)
% h = PlaneDubins_plotAbsRS(vehicle, g, data, extraArgs)
%     Plots an absolute reachable set (eg. forward or backward reachable set for
%     a Plane or a DubinsCar
%
% Inputs:
%     vehicle   - vehicle for which plotting is done
%     g, data   - grid and value function on the grid
%     extraArgs - plotting parameters; see visSetIm
% 
% Output:
%     h - handle of plotted object

if nargin < 4
  extraArgs.LineStyle = ':';
  extraArgs.LineWidth = 2;
end

[g2D, data2D] = proj(g, data, [0 0 1]);
dataRot = rotateData(g2D, data2D, vehicle.x(3), [1 2], []);
gShift = shiftGrid(g2D, vehicle.x(1:2));

h = visSetIm(gShift, dataRot, 'r', 0, extraArgs);
end