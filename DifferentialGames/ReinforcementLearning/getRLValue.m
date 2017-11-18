function value = getRLValue(x, w)
% get RL value prediction from state and weights

% linear predictor
value = dot(w, getRLFeatures(x));
