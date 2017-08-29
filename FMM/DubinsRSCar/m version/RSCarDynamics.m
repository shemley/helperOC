function dy = RSCarDynamics(t,y)

global forward  % 1 or -1
global turn     % 1, 0, or -1
dy = zeros(3,1);
dy(1) = -forward*cos(y(3));
dy(2) = -forward*sin(y(3));
dy(3) = -turn;