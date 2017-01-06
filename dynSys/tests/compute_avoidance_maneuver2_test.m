function compute_avoidance_maneuver2_test()

load('PlaneCAvoid_test_adversarial.mat')
load('Plane_test2.mat')

tic
[trajA, trajB] = compute_avoidance_maneuver2(safety_set, goal_sat_set); 
toc

figure
plot(trajA(1,:), trajA(2,:), 'b.')
hold on
plot(trajB(1,:), trajB(2,:), 'r.')

axis equal
grid on
end