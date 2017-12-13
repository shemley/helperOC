function intervalIndices = getIntervalIndices(x,stateMin,intervalSize,numIntervals)


if ~iscolumn(x)
    x = x';
end

if ~iscolumn(stateMin)
    stateMin = stateMin';
end

if ~iscolumn(intervalSize)
    intervalSize = intervalSize';
end

rawIndices = (x-stateMin)./intervalSize;
tooSmall = rawIndices < 0;
tooBig = rawIndices > numIntervals;
rawIndices(tooSmall) = 1;
rawIndices(tooBig) = numIntervals;    

intervalIndices =  ceil(rawIndices);