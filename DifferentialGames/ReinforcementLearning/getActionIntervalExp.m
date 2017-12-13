function u = getActionIntervalExp(x,dt,dynSys,mode,intervalData,intervalSize,stateMin,numIntervals)
% get action from interval exploration

if strcmp(mode,'min')
    attacker = true;
elseif strcmp(mode,'max')
    attacker = false;
else
    error('Invalid mode input. Must be ''max'' or ''min''.')
end

% State updates for each player
attackerUpdate = @(v,u) v + intervalSize.*[u; 0; 0]; % was dt instead of interval size. increased to ensure that new bins are evaluated
defenderUpdate = @(v,d) v + intervalSize.*[0; 0; d];

if attacker
    % Attacker first
    agentMax  = dynSys.uMax;
    oppMax = dynSys.dMax;  
    agentUpdate = attackerUpdate;
    oppUpdate = defenderUpdate;
    best = Inf;
else 
    % Defender first
    agentMax = dynSys.dMax;
    oppMax = dynSys.uMax;
    agentUpdate = defenderUpdate;
    oppUpdate = attackerUpdate;
    best = -Inf;
end

% Define list of actions for agent
agentActions = getRLLegalActions(agentMax);

numActions = size(agentActions,2);
count = 1;
bestIndices = zeros(size(agentActions));
for i = 1:numActions
    newX = agentUpdate(x,agentActions(:,i));
    indices = getIntervalIndices(newX,stateMin,intervalSize,numIntervals);
    if attacker
        intLower = intervalData{indices(1),indices(2),indices(3),indices(4)}.lower;
        if intLower < best
            count = 1;
            best = intLower;
            bestIndices(count) = i;
        elseif intLower == best
            count = count + 1;
            bestIndices(count) = i;
        end
    else
        intUpper = intervalData{indices(1),indices(2),indices(3),indices(4)}.upper;
        if intUpper > best
            count = 1;
            best = intUpper;
            bestIndices(count) = i;
        elseif intUpper == best
            count = count + 1;
            bestIndices(count)  = i;
        end
    end
end

u = agentActions(:,bestIndices(randi(count)));

