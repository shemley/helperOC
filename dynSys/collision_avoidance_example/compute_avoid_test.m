function compute_avoid_test(visualize)

if nargin < 1
  visualize = true;
end

load('safe_set_adv.mat')
load('goal_sat_set.mat')

N = 100; % Number of instances to test

if visualize
  figure
  spC = ceil(sqrt(N))+1;
  spR = ceil(N/spC);
end

planeA.wMax = 1;
planeA.vrange = [5 5];
planeA.dMax = [0 0];

planeB.wMax = 1;
planeB.vrange = [5 5];
planeB.dMax = [0 0];

comp_time = 0;
for i = 1:N
  fprintf('%d of %d\n', i, N)
  
  L = 10;
  thetaA = 0.55*pi + 0.9*pi*rand;
  planeA.x = [L*cos(thetaA); L*sin(thetaA); thetaA+pi];
  
  thetaB = -0.45*pi + 0.9*pi*rand;
  planeB.x = [L*cos(thetaB); L*sin(thetaB); thetaB+pi];  
  
  tic
  [trajA, trajB] = compute_avoid_man(safety_set, goal_sat_set, planeA, ...
    planeB, false);
  comp_time = comp_time + toc;
  
  if visualize
    subplot(spR, spC, i)
    plot(trajA(1,:), trajA(2,:), 'b.')
    hold on
    plot(trajB(1,:), trajB(2,:), 'r.')
    axis equal
    grid on
    drawnow
  end
end

fprintf('Total computation time for %d trials: %.2f seconds\n', N, comp_time)
end