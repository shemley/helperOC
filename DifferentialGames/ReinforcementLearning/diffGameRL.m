function w = diffGameRL(map, tMax, dt, captureRadius, uMax, dMax)

% Seed random number generator
rng shuffle

% Define dynamic system
% obj = KinVehicleND2Agent(x, uMax, dMax)
dynSys = KinVehicleND2Agent([0,0,0,0], uMax, dMax);
uLegal = getRLLegalActions(uMax);
dLegal = getRLLegalActions(dMax);

% Initialize random weights
w = rand(size(getRLFeatures([0;0;0;0])));

% Create time vector
tau = 0:dt:tMax;

% Keep track of number of iterations
numIter = 1;
maxIters = 100000;

% Learning parameters
discount = 0.7;
lambda = 0.7;
regularizationRate = 0.01;

while numIter <= maxIters
    
    fprintf('Running Iteration %i.\n',numIter)
    
    % Initialize random state in state space
    stateMin = repmat(map.min, 2, 1);
    stateRange = repmat(map.max - map.min, 2, 1);
    
    dynSys.x = rand(4,1).*stateRange + stateMin;
    
    % Pre-allocate trajectory matrices
    stateTraj = nan(4,length(tau));
    rewardTraj = nan(size(tau));
    
    % Set first values
    stateTraj(:,1) = dynSys.x;
    rewardTraj(1) = 0;
        
    % Start at 2nd step
    step = 2;
    
    % Run episode to completion
    while step <= length(tau) && ~isDiffGameEnd(map,dynSys.x,captureRadius)
        epsilon = getEpsilon(numIter);       
        
        % Get attacker control (using epsilon-greedy)
        if rand(1) > epsilon
            u = getRLAction(dynSys.x, w, dt, dynSys, 'min');
        else
            u = uLegal(:,randi(length(uLegal),1));
        end
            
        % Get defender control (using epsilon-greedy)
        if rand(1) > epsilon
            d = getRLAction(dynSys.x, w, dt, dynSys, 'max');
        else
            d = dLegal(:,randi(length(dLegal),1));
        end
        
        % Update state with these actions
        dynSys.updateState(u, dt, dynSys.x, d);
        
        % Store new state
        stateTraj(:,step) = dynSys.x;
        
        % Store new reward
        rewardTraj(step) = getRLReward(map, dynSys.x, dt, captureRadius);
        
        % Increment step
        step = step + 1;        
    end    
    
    % Remove unused indices from trajectories
    stateTraj(:,step:end) = [];
    rewardTraj(step:end) = [];
    
    % If there are at least two trajectory steps, propagate backward from
    % the last state and update the weights (SGD for TD learning/TD-lambda)
    if size(stateTraj,2) > 1
        % Compute value function at final point on state trajectory
        nextVal = getRLValue(stateTraj(:,end), w);
        
        % Get step size
        eta = getStepSize(numIter);
        for i = (size(stateTraj,2)-1):-1:1
            thisVal = getRLValue(stateTraj(:,i), w);
            r = rewardTraj(i);
            featureVec = getRLFeatures(stateTraj(:,i));
            
            % Update weights
            targetVal = (r + discount*nextVal);
            regTerm = regularizationRate*w;
            w = w - eta*((thisVal - targetVal)*featureVec + regTerm);
            
            % Update next value with target decayed by a factor of lambda
            nextVal = lambda*targetVal;
        end
        
        % Increment number of iterations only if w was updated
        numIter = numIter + 1;
    end
    
    if any(numIter == [1,10,50,100,200,500,1000,2000,5000,10000,20000,50000,100000])
        save(['./DifferentialGames/ReinforcementLearning/Data/Weights_Iter',num2str(numIter),'.mat'],'w')
    end
end


% Function to get step size
function eta = getStepSize(numIter)

eta = 1/sqrt(numIter);

% Function to epsilon for epsilon greedy
function epsilon = getEpsilon(numIter)

epsilon = numIter^(-.375);

