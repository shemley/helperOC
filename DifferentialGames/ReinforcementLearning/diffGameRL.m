function w = diffGameRL(map, tMax, dt, captureRadius, uMax, dMax, options)

% Seed random number generator
rng shuffle

% Define dynamic system
% obj = KinVehicleND2Agent(x, uMax, dMax)
dynSys = KinVehicleND2Agent([0,0,0,0], uMax, dMax);
uLegal = getRLLegalActions(uMax);
dLegal = getRLLegalActions(dMax);

% Create time vector
tau = 0:dt:tMax;

% Keep track of number of iterations
numIter = 1;
maxIters = 10000000;

% Learning parameter defaults
discount = 0.7;
lambda = 0.7;
regularizationRate = 0.01;

nnet = false;
interval = false;
numIntervals = 10;
options.map = map;
alpha = 0.05;
sweep = false;
sig = true;
setData = false;
if nargin > 6   
    if isfield(options,'discount')
        discount = options.discount;
    end
    if isfield(options,'lambda')
        lambda = options.lambda;
    end
    if isfield(options,'regularizationRate')
        regularizationRate = options.regularizationRate;
    end
    if isfield(options,'sig')
        sig = options.sig;
    end
    if isfield(options,'setData')
        setData = options.setData;
    end
    if isfield(options,'nnet')
        nnet = options.nnet;
        
        hiddenLayers = 1;
        nodesPerLayer = 4;      
        if isfield(options,'hiddenLayers')
            hiddenLayers = options.hiddenLayers;
        end
        if isfield(options,'nodesPerLayer')
            nodesPerLayer = options.nodesPerLayer;
        end          
    end
    
    if isfield(options,'interval')
        interval = options.interval;
    end
    if isfield(options,'numIntervals')
        numIntervals = options.numIntervals;
    end
    if isfield(options,'alpha')
        alpha = options.alpha;
    end
    if isfield(options,'sweep')
        sweep = options.sweep;
    end
end

% if nnet, build neural network
if nnet
    neuralNet = NeuralNetwork(length(getRLFeatures([0;0;0;0],options)),...
                            hiddenLayers,nodesPerLayer,regularizationRate);
    w = neuralNet;    
else
    % Initialize random weights
    w = rand(size(getRLFeatures([0;0;0;0],options)));
end




    
% Define state space range
stateMin = repmat(map.grid_min, 2, 1);
stateRange = repmat(map.grid_max - map.grid_min, 2, 1);


% If interval, set up bins to keep track of distributions. Initialize with
% mean of zero and large variance
intervalSize = stateRange./numIntervals;
if interval    
    intervalData = cell(numIntervals,numIntervals,numIntervals,numIntervals);
    intervalStruct.mean  = 0;
    intervalStruct.var   = 150^2;
    intervalStruct.n     = 2;
    t_crit = tinv(1-alpha/2,intervalStruct.n-1);
    width = t_crit*sqrt(intervalStruct.var/intervalStruct.n);
    intervalStruct.lower = intervalStruct.mean - width;
    intervalStruct.upper = intervalStruct.mean + width;
    
    for i1 = 1:numIntervals
      for i2 = 1:numIntervals
        for i3 = 1:numIntervals
          for i4 = 1:numIntervals
            intervalData{i1,i2,i3,i4} = intervalStruct;
          end
        end
      end
    end
end

% Set up prioritized sweeping
if sweep
    capacity = numIntervals^4;
    priorityQ = PriorityQueue(capacity,'max');
    qStruct.priority = 50;
    qStruct.value = 0;
    qStruct.stateRange = intervalSize;
    for i1 = 1:numIntervals
      for i2 = 1:numIntervals
        for i3 = 1:numIntervals
          for i4 = 1:numIntervals
            qStruct.stateMin = stateMin + ([i1;i2;i3;i4]-1).*intervalSize;
            priorityQ.add(qStruct);
          end
        end
      end
    end
end


while numIter <= maxIters
    
    fprintf('Running Iteration %i.\n',numIter)
    
    % Init random state in state space. Based on sweeping priority if sweep
    if sweep
        qStruct = priorityQ.dequeue();
        if ~isempty(qStruct)
            dynSys.x = rand(4,1).*qStruct.stateRange + qStruct.stateMin;
        else
            dynSys.x = rand(4,1).*stateRange + stateMin;
        end
    else
        dynSys.x = rand(4,1).*stateRange + stateMin;
    end
        
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
        
        
        if interval            
            u = getActionIntervalExp(dynSys.x,dt,dynSys,'min',intervalData,intervalSize,stateMin,numIntervals);
            d = getActionIntervalExp(dynSys.x,dt,dynSys,'max',intervalData,intervalSize,stateMin,numIntervals);
        else
            epsilon = getEpsilon(numIter);       

            % Get attacker control (using epsilon-greedy)
            if rand(1) > epsilon
                u = getRLAction(dynSys.x, w, dt, dynSys, 'min',options);
            else
                u = uLegal(:,randi(length(uLegal),1));
            end

            % Get defender control (using epsilon-greedy)
            if rand(1) > epsilon
                d = getRLAction(dynSys.x, w, dt, dynSys, 'max',options);
            else
                d = dLegal(:,randi(length(dLegal),1));
            end        
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
        if nnet
            featureVec = getRLFeatures(stateTraj(:,end),options);
            nextVal = neuralNet.calculateValue(featureVec);
        else
            nextVal = getRLValue(stateTraj(:,end), w, options);
        end
            
        % Get step size
        eta = getStepSize(numIter);
        for i = (size(stateTraj,2)-1):-1:1            
            r = rewardTraj(i);
            featureVec = getRLFeatures(stateTraj(:,i),options);
            
            % Update weights            
            targetVal = (r + discount*nextVal);
            
            % If interval, determine bin and update
            if interval || sweep
                indices = getIntervalIndices(stateTraj(:,i),stateMin,intervalSize,numIntervals);
            end
            
            if interval                
                intervalData{indices(1),indices(2),indices(3),indices(4)} =...
                    updateInterval(intervalData{indices(1),indices(2),indices(3),indices(4)}...
                                   ,targetVal,alpha);
            end
            
            
            if nnet
                neuralNet.backprop(featureVec, targetVal, eta);
                thisVal = neuralNet.getValue();
            else            
                thisVal = getRLValue(stateTraj(:,i), w, options);
                regTerm = regularizationRate*w;
                w = w - eta*((thisVal - targetVal)*featureVec + regTerm);
            end
            
            % update sweep priorities
            if sweep
                qStruct.stateMin = stateMin + (indices-1).*intervalSize;
                qStruct.priority = abs(thisVal-targetVal);
                qStruct.value = targetVal;
                qStruct.stateRange = intervalSize;
                priorityQ.add(qStruct);
            end
                
            % Update next value with target decayed by a factor of lambda
            nextVal = lambda*targetVal;
        end
        
        % Increment number of iterations only if w was updated
        numIter = numIter + 1;
    end
    
    if any(numIter == [1,5,10,20,50,75,100,200,500,1000,1500,2000,2500,...
            3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000,8500,...
            9000,9500,10000,11000,12000,13000,14000,15000,17000,20000,25000,...
            30000,35000,40000,45000,50000,60000,70000,80000,90000,100000,150000,...
            200000,300000,500000,800000,1000000,2000000,10000000])

        
        if nnet
            filename = sprintf(...
                './DifferentialGames/ReinforcementLearning/Data/Nnet_%s_Int%u_a%.2f_Sw%u_Sig%u_Obs%u_HL%u_Nodes%u_Reg%.2f_Iter%i.mat',...
                map.name,interval,alpha,sweep,sig,setData,hiddenLayers,nodesPerLayer,regularizationRate,numIter);
                
            save(filename,'neuralNet','options')
        else
            filename = sprintf(...
                './DifferentialGames/ReinforcementLearning/Data/Weights_%s_Int%u_a%.2f_Sw%u_Sig%u_Obs%u_Reg%.2f_Iter%i.mat',...
                map.name,interval,alpha,sweep,sig,setData,hiddenLayers,nodesPerLayer,regularizationRate,numIter);
            save(filename,'w','options')
        end
    end
end


% Function to get step size
function eta = getStepSize(numIter)

eta = 0.1/sqrt(numIter);

% Function to epsilon for epsilon greedy
function epsilon = getEpsilon(numIter)

epsilon = numIter^(-.375);

