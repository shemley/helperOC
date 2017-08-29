function [i1, i2] = xy2inds(xin,yin,g)
% [i1, i2] = xy2inds(xin,yin,grid)
% converts (x,y) coordinate to grid indices
%
% Inputs:
%   (xin,yin)     - coordinate to convert
%   g             - grid structure
%
% Outputs
%   (i1, i2)    - Indices
% 
% Mo Chen, Sept. 26, 2013
if length(xin)~=length(yin)
    error('x and y must have the same length!')
end

i1 = zeros(size(xin));
i2 = zeros(size(yin));

for i = 1:length(xin)
    [~, i1(i)] = min(abs(g.vs{1} - xin(i)));
    [~, i2(i)] = min(abs(g.vs{2} - yin(i)));
end

end