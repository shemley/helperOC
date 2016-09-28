function [gIm, dataIm] = MIE2Implicit(gMIE, dataMIE, side, gTI)
% [gIm, dataIm] = MIE2Implicit(gMIE, dataMIE, side, gTI)
%     Converts an MIE upper or lower function to a full-implicit function
%
% Inputs:
%     gMIE, dataMIE: grid and MIE function
%     side:          "side" of the MIE function, must be 'upper' or 'lower'
%     gTI:           grid in the terminal integrator dimension (used to
%                    construct full grid)
%
% Outputs:
%     gIm, dataIm:  grid and fully-implicit function

if ~strcmp(side, 'lower') && ~strcmp(side, 'upper')
  error('side must be ''lower'' or ''upper''!')
end

%% Create full grid using MIE grid and terminal integrator grid
if nargin < 4
  gTI.dim = 1;
  gTI.N = 51;
  gTI.min = -10;
  gTI.max = 10;
  gTI.bdry = @addGhostExtrapolate;
end

gIm.dim = gTI.dim + gMIE.dim;
gIm.N = [gTI.N; gMIE.N];
gIm.min = [gTI.min; gMIE.min];
gIm.max = [gTI.max; gMIE.max];
gIm.bdry = gTI.bdry;
for i = 1:length(gMIE.bdry)
  gIm.bdry{end+1,1} = gMIE.bdry{i};
end
gIm = processGrid(gIm);

if nargout < 2
  return
end

%% Create implicit value function
if strcmp(side, 'lower')
  dataIm = backProj(gIm, dataMIE, 2:gIm.dim) - gIm.xs{1};
else
  dataIm = gIm.xs{1} - backProj(gIm, dataMIE, 2:gIm.dim);
end

end