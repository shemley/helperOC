function phi = getRLFeatures(x,options)
% function to return feature vector for RL given state
% if options.obs is true, options.map must exist

sig = true;
setData = false;
obs = false;
nnet = false;
map = options.map;
if nargin > 1
    if isfield(options,'sig')
        sig = options.sig;
    end
    if isfield(options,'setData')
        setData = options.setData;
    end
    if isfield(options,'nnet')
        nnet = options.nnet;
    end
end

if setData
    % handle positions outside of the grid
    smallNum = 0.05;
    roundedX = x;
    tooBig = roundedX > [map.grid_max;map.grid_max] - smallNum;
    tooSmall = roundedX < [map.grid_min;map.grid_min] + smallNum;
    
    roundedX(tooBig) = map.grid_max(1)-smallNum;
    roundedX(tooSmall) = map.grid_min(1)+smallNum;
    
    targetVal = eval_u(map.g,map.target.data,roundedX(1:2));
    if isfield(map,'obstacles') 
        obs = true;
        attObsVal = eval_u(map.g,map.obstacles,roundedX(1:2));
        defObsVal = eval_u(map.g,map.obstacles,roundedX(3:4));
    end 
end

if ~nnet
    % Initialize 6 features
    phi = zeros(6,1);

    % Features in states
    phi(1) = x(1);
    phi(2) = x(2);
    phi(3) = x(3);
    phi(4) = x(4);

    % Distance features
    phi(5) = norm(x(1:2) - x(3:4),2); % Distance between agents
    phi(6) = norm(x(1:2),2); % Distance of attacker to target
    
    if setData
        phi(7) = targetVal;
        if obs
            phi(8) = attObsVal;
            phi(9) = defObsVal;
        end
    end
    
    if sig
        phi = 1./(1+exp(-phi));
    end
else
    % use neural network with simple features. Include target/obs set data
    % if requested
    if iscolumn(x)
        phi = [1;x];
        if setData
            phi = [phi; targetVal]; 
        end        
        if obs
            phi = [phi; attObsVal; defObsVal];
        end
    else
        phi = [1,x];
        if setData
            phi = [phi, targetVal]; 
        end        
        if obs
            phi = [phi, attObsVal, defObsVal];
        end
    end
end