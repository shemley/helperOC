function action = getRLAction(x, w, dynSys, mode)
% function to get action based on RL value. Uses a 1 level depth limited
% search with eval function. 
% Mode determines whose turn it is first. 'min' means attacker first, 'max'
% means defender first

if strcmp(mode,'min')
    attacker = true;
elseif strcmp(mode,'max')
    attacker = false;
else
    error('Invalid mode input. Must be ''max'' or ''min''.')
end

% State updates for each player
attackerUpdate = @(v,u) v + [u; 0; 0];
defenderUpdate = @(v,d) v + [0; 0; d];

if attacker
    % Attacker first
    firstMax  = dynSys.uMax;
    secondMax = dynSys.dMax;  
    firstUpdate = attackerUpdate;
    secondUpdate = defenderUpdate;
else 
    % Defender first
    firstMax = dynSys.dMax;
    secondMax = dynSys.uMax;
    firstUpdate = defenderUpdate;
    secondUpdate = attackerUpdate;
end

% Define list of actions for each player
firstActions = getRLLegalActions(firstMax);
secondActions = getRLLegalActions(secondMax);

% Set up bounds on value function
lower = -inf;
upper = inf;

% Loop through all possible actions
for i = 1:size(firstActions,2)
    firstAct = firstActions(:,i);
    newX = firstUpdate(x,firstAct);
    
    if attacker
        maxVal = -inf;
    else
        minVal = inf;
    end
    
    for j = 1:size(secondActions,2)
        secondAct = secondActions(:,j);
        newX = secondUpdate(newX,secondAct);
        
        newValue = getRLValue(newX, w);
        
        if attacker
            if newValue > upper
                break;
            elseif newValue > maxVal
                maxVal = newValue;
            end  
        else
            if newValue < lower
                break;
            elseif newValue > minVal
                minVal = newVal;
            end
        end            
    end
    
    if attacker
        if maxVal < upper
            upper = maxVal;
            action = firstAct;
        end
    else
        if minVal > lower
            lower = minVal;
            action = firstAct;
        end
    end        
end