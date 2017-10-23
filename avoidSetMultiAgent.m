function data = avoidSetMultiAgent(grid, radii, nagents, ndims, ignoreDims)
% avoidSetMultiAgent: implicit surface function for avoid set for
%                     multi-agent interactions
%
%   data = avoidSetMultiAgent(grid, nagents, ndims, radii, ignoreDims)
%
% Creates an implicit surface function (actually signed distance) for the
% distance between each agent and all others, with a specified radius for
% each
%
% Can be used to create:
%   Intervals, circles and spheres (if ignoreDims is empty).
%   Slabs (if ignoreDims contains all dimensions except one).
%
% parameters:
% Input Parameters:
%
%   grid: Grid structure (see processGrid.m for details).
%         grid.dim must be equal to nagents*ndims
%   
%   radii: a vector of capture/avoid radii for each agent. Defaults to 1.
%   When two agents' positions are compared, the smaller of their two radii
%   is used
%
%   nagents: number of agents' state vectors represented in the grid.
%   Defaults to 2
%
%   ndims: number of dimensions in each agent's state vector. Defaults to
%   grid.dim/nagents
%
%   ignoreDims: vector specifying indices of states to ignore in agent
%   comparison. The index is for an individual agent's state vector and
%   therefore ranges from 1 to ndims. Defaults to [] (none)
%
% Output Parameters:
%
%   data: Output data array (of size grid.size) containing the implicit
%   surface function.
%
% Adapted from shapeCylinder.m
% Scott Hemley 2017-10-22

%---------------------------------------------------------------------------
% Default parameter values.
if(nargin < 5)
  ignoreDims = [];
end

% default to 2 agents
if(nargin < 3)
  nagents = 2;
end

% each agent must have the same number of dimensions
if(nargin < 4)
  ndims = grid.dim/nagents;
end

if(nargin < 2)
  radii = ones(nagents,1);
end

% if one radius is input, apply to all agents
if numel(radii) == 1
  radii = radii*ones(nagents,1);
end

% if radii is a row vector, transpose
if ~iscolumn(radii)
  radii = radii';    
end

% Check for compatible and integer valued ndims/nagents
if (ndims*nagents ~= grid.dim) || (rem(ndims,1) ~= 0) || (rem(nagents,1) ~= 0) 
  error('Each agent must have the same number of dimensions!')
end

%---------------------------------------------------------------------------
% Signed distance function calculation.
data = -inf(grid.shape);
% Loop through agents
for i = 1:(nagents-1)
  for j = 2:(nagents)
    radius = max(radii(i),radii(j)); % use larger radius of the 2 agents
    pairData = zeros(grid.shape); % data to compare a new pair of agents
    for k = 1:ndims
        if(all(k ~= ignoreDims))
            iDim = (i - 1)*ndims + k;
            jDim = (j - 1)*ndims + k;
            pairData = pairData + (grid.xs{iDim} - grid.xs{jDim}).^2;
        end
    end
    pairData = sqrt(pairData) - radius; % find points within radius (>0)
    data = max(data,pairData); % take maximum of data and latest pair
  end
end

%---------------------------------------------------------------------------
% Warn the user if there is no sign change on the grid
%  (ie there will be no implicit surface to visualize).
if(all(data(:) < 0) || (all(data(:) > 0)))
  warning([ 'Implicit surface not visible because function has ' ...
            'single sign on grid' ]);
end
