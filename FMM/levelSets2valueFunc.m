function u = levelSets2valueFunc(g2D, levelSets, levels)
% u = levelSets2valueFunc(g2D, levelSets, levels)
% Mo Chen, 2013-12-03

large = 1000; % large number
u = large*ones(g2D.N');

u(levelSets{1} <=0) = levels(1);
for i = 2:length(levels)
    leftBehindSet = shapeDifference(u-levels(i-1), levelSets{i});
    u( leftBehindSet <= 0) = large;
    
    % Set surrounding grid points to the next level
    u( levelSets{i}<=0 & u>levels(i-1) ) = levels(i);
end

end