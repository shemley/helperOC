function [obsTotal, vMax] = plopObstacles(obsBRS_filenames, positions, ...
  angles, global_g)

if ~iscell(obsBRS_filenames)
  obsBRS_filenames = {obsBRS_filenames};
end

% Initialize total obstacle to empty
obsTotal = inf(global_g.N');
if nargout > 1
  vMax = inf;
end

for i = 1:length(obsBRS_filenames)
  load(obsBRS_filenames{i})
  
  for j = 1:length(positions{i})
    position = [positions{i,j}(1) 0 positions{i,j}(2) 0];
    angle = angles{i,j};
    
    dataRotated = rotateData(sD.grid, data, angle, [1 3], []);
    gShifted = shiftGrid(sD.grid, position);
    
    obsiTransformed = migrateGrid(gShifted, dataRotated, global_g);
    obsTotal = min(obsTotal, obsiTransformed);
    
    if nargout > 1
      vMax = min(vMax, get_vMax(global_g, obsTotal, obsiTransformed));
    end
  end
end

end

function vMax = get_vMax(g, BRS1, BRS2)
% Determines the minimum speed such that the two backward reachable sets do not
% intersect

if all(isinf(BRS1(:))) || all(isinf(BRS2(:)))
  vMax = min(abs([g.min(2) g.max(2) g.min(4) g.max(4)]));
  return
end

%% Binary search to find vMax
maxGoodIter = 5;
goodIter = 0;

upper = min(abs([g.min(2) g.max(2) g.min(4) g.max(4)]));
lower = 0;

while goodIter < maxGoodIter
  vMax = 0.5*(upper+lower)
  % Ignore indices which exceed vMax
  inds_over_vMax = g.xs{2}.^2 + g.xs{4}.^2 > vMax^2;
  BRS1(inds_over_vMax) = inf;
  BRS2(inds_over_vMax) = inf;

  % Project to 2D
  [~, BRS1_proj] = proj(g, BRS1, [0 1 0 1]);
  [~, BRS2_proj] = proj(g, BRS2, [0 1 0 1]);
  
  % Check for intersection
  if any(max(BRS1_proj(:), BRS2_proj(:)) < 0)
    % Overlap detected; current vMax is high
    upper = vMax;
  else
    % No overlap; current vMax is OK
    lower = vMax;
    goodIter = goodIter + 1;
  end
end



end