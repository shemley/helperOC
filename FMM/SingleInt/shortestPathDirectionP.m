function dirn = shortestPathDirectionP(g, P, x)
% dirn = shortestPathDirectionP(g, P, x)
% Computes optimal direction along the shortest path
%
% Uses costate information instead of value information in order to use Ian
% Mitchell's routines for high order spatial derivatives.
%
% 

% Gradient
p = calculateCostate(g,P,x);

small = 1e-9;
if norm(p)>small,   dirn = -p / norm(p);
else                dirn = [0 0];
end
    
end