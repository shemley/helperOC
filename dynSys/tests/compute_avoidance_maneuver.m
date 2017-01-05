function [trajA, trajB] = compute_avoidance_maneuver(safety_set, ...
  goal_satisf_set, planeA, planeB)

if nargin < 3
  %% Vehicle parameters
  xA = [-10; 0; 0];
  wMaxA = 1; % Maximum turn rate (rad/s)
  vRangeA = [5 5]; % Speed range (m/s)
  dMaxA = [0 0]; % Disturbance (see PlaneCAvoid class)
  
  wMaxB = 1;
  vRangeB = [5 5];
  dMaxB = [0 0];
  xB = [10; 0.5; pi];
  
else
  xA = planeA.x;
  wMaxA = planeA.wMax;
  vRangeA = planeA.vrange;
  dMaxA = planeA.dMax;
  
  xB = planeB.x;
  wMaxB = planeB.wMax;
  vRangeB = planeB.vrange;
  dMaxB = planeB.dMax;
end

planes = {Plane(xA, wMaxA, vRangeA, dMaxA); Plane(xB, wMaxB, vRangeB, dMaxB)};
planeRel = PlaneCAvoid(planes);
rel_x = cell(2,1);
safety_vals = cell(2,1);
goal_vals = cell(2,1);
init_headings = {xA(3); xB(3)};

%% Simulate to obtain trajectories
dt = 0.1;
t = 0:dt:20;
small = 1e-2;
ti_min = 10; % Minimum number of steps to simulate

for ti = 1:length(t)
  for veh = 1:length(planes)
    % Compute safety value
    rel_x{veh} = PlaneDubins_relState(planes{veh}.x, planes{other(veh)}.x);
    safety_vals{veh} = eval_u(safety_set.g, safety_set.data, rel_x{veh});
    
    if safety_vals{veh} > small
      % If safe, use goal satisfaction controller
      x = rotate_to_goal_satisf_frame(planes{veh}.x, init_headings{veh});
      p = eval_u(goal_satisf_set.g, goal_satisf_set.deriv, x);
      u = planes{veh}.optCtrl([], x, p, 'min');
      goal_vals{veh} = eval_u(goal_satisf_set.g, goal_satisf_set.data, x);
      
    else
      % If not safe, use safety controller
      p = eval_u(safety_set.g, safety_Set.deriv, rel_x{veh});
      u = planeRel.optCtrl([], rel_x{veh}, p, 'max');
      
    end
    
    % Update state
    planes{veh}.updateState(u, dt);
    
    planes{veh}.plotPosition();
  end
  
  drawnow
  % Break if all vehicles have gotten to target
  if ti > ti_min && all([safety_vals{:}] > small) && all([goal_vals{:}] < small)
    break
  end
end

trajA = planes{1}.xhist;
trajB = planes{2}.xhist;
end

function xOut = rotate_to_goal_satisf_frame(xIn, init_heading)

xOut = zeros(3,1);
xOut(1:2) = rotate2D(xIn(1:2), -init_heading);
xOut(3) = xIn(3) - init_heading;

end

function ind_out = other(ind_in)

if ind_in == 1;
  ind_out = 2;
else
  ind_out = 1;
end

end