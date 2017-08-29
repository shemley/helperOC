function df = Dubinfun(t,f)
% Function for computing trajectories
% Implements equations 18 and 19
global x
global y
global theta
global rmin
global utheta

df = zeros(3,1);

% partial derivative at this point
uthetas = interpn(x,y,theta,utheta, f(1), f(2), f(3));

% equations 18 and 19
df(1) = -cos(f(3));
df(2) = -sin(f(3));
df(3) = -sign(uthetas)/rmin;