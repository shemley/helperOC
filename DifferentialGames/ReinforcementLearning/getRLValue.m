function value = getRLValue(x, w, options)
% get RL value prediction from state and weights

% linear predictor
value = dot(w, getRLFeatures(x,options));
