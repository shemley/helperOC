function [trajA, trajB] = compute_avoid_man(safety_set, goal_sat_set, ...
  planeA, planeB, visualize)
% [trajA, trajB] = compute_avoid_man(safety_set, goal_sat_set, planeA, ...
%   planeB, visualize)
%     Computes the trajectories from avoidance maneuvers for two planes
%
% Inputs:
%     safety_set: Safety reachable set
%     goal_sat_set: Goal satisfaction reachable set for getting back to original
%                   path
%     planeA, planeB: structs with vehicle parameters
%         .x:      current state
%         .wMax:   maximum turn rate
%         .vrange: speed range (use eg. [5 5] for constant speed)
%         .dMaxB:  disturbance (see Plane class)
%
% Outputs:
%     trajA, trajB: list of 3D points resulting from avoidance
%
% Mo Chen, 2017-01-06

if nargin < 3 || isempty(planeA) || isempty(planeB)
  %% Vehicle parameters
  L = 10;
  xA = [-L; 0; 0];
  wMaxA = 1; % Maximum turn rate (rad/s)
  vRangeA = [5 5]; % Speed range (m/s)
  dMaxA = [0 0]; % Disturbance (see PlaneCAvoid class)
  
  thetaB = 3*pi/4*rand;
  xB = [L*cos(thetaB); L*sin(thetaB); thetaB+pi];
  wMaxB = 1;
  vRangeB = [5 5];
  dMaxB = [0 0];
  
  
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

if nargin < 5
  visualize = true;
end

x0 = {xA; xB};

planes = {Plane(xA, wMaxA, vRangeA, dMaxA); Plane(xB, wMaxB, vRangeB, dMaxB)};
planeRel = PlaneCAvoid(planes);

init_headings = {xA(3); xB(3)};
heading_errors = cell(2,1);
position_errors = cell(2,1);


%% Simulate to obtain trajectories
dt = 0.1;
t = 0:dt:200;

evader_ind = nan;
avoidance_started = false;
deviated_from_path = false;

if visualize
  extraArgs.ArrowLength = 2;
  figure
end

for ti = 1:length(t)
  [u, avoidance_started, evader_ind] = control_logic(planes, planeRel, ...
    init_headings, avoidance_started, evader_ind, safety_set, goal_sat_set);
  
  for veh = 1:length(planes)
    % Update state
    planes{veh}.updateState(u{veh}, dt);
    
    % Check errors
    heading_errors{veh} = abs(init_headings{veh} - planes{veh}.x(3));
    n = [cos(x0{veh}(3)); sin(x0{veh}(3))];
    position_errors{veh} = dist_pt_to_line(x0{veh}(1:2), planes{veh}.x(1:2), n);
  end
  
  if visualize
    for veh = 1:length(planes)
      planes{veh}.plotPosition(extraArgs);
    end
    if ti == 1
      xlim([-30 30])
      ylim([-30 30])
      axis square
      grid on
    end
    drawnow
  end
  
  % Break if all vehicles have gotten to target
  if deviated_from_path
    max_pos_err = max(position_errors{:});
    if max_pos_err <= goal_sat_set.g.dx(2)
      max_heading_err = max(heading_errors{:});
      if max_heading_err <= goal_sat_set.g.dx(3)      
        break
      end
    end
  else
    max_pos_err = max(position_errors{:});
    if max_pos_err > goal_sat_set.g.dx(2)
      max_heading_err = max(heading_errors{:});
      if max_heading_err > goal_sat_set.g.dx(3)
        deviated_from_path = true;
      end
    end
  end
end

trajA = planes{1}.xhist;
trajB = planes{2}.xhist;
end

function [u, avoidance_started, evader_ind] = control_logic(planes, ...
  planeRel, init_headings, avoidance_started, evader_ind, safety_set, ...
  goal_sat_set)

small = 1e-2;
u = cell(2,1);

% Compute safety value
if avoidance_started
   rel_x = ...
     PlaneDubins_relState(planes{evader_ind}.x, planes{other(evader_ind)}.x)';

  if any(rel_x < safety_set.g.min) || any(rel_x > safety_set.g.max)
    safe = true;
  else
    safety_val = eval_u(safety_set.g, safety_set.data, rel_x);
    if safety_val > small
      safe = true;
    else
      safe = false;
    end
  end
  
  if safe
    for veh = 1:2
      % If safe, use goal satisfaction controller
      x = rotate_to_goal_sat_frame(planes{veh}.x, init_headings{veh});
%       tEarliest = find_earliest_BRS_ind(goal_sat_set.g, goal_sat_set.data, x);
%       Deriv = {goal_sat_set.deriv{1}(:,:,:,tEarliest); ...
%         goal_sat_set.deriv{2}(:,:,:,tEarliest); ...
%         goal_sat_set.deriv{2}(:,:,:,tEarliest)};
%       p = eval_u(goal_sat_set.g, Deriv, x);
      p = eval_u(goal_sat_set.g, goal_sat_set.TTRderiv, x);
      u{veh} = planes{veh}.optCtrl([], x, p, 'min');
    end
  else
    p = eval_u(safety_set.g, safety_set.deriv, rel_x);
    u{evader_ind} = planeRel.optCtrl([], rel_x, p, 'max');
    
    u{other(evader_ind)} = u{evader_ind};
    
  end
else
  for veh = 1:2
    rel_x = cell(2,1);
    safety_vals = cell(2,1);    
    rel_x{veh} = PlaneDubins_relState(planes{veh}.x, planes{other(veh)}.x)';
    
    if any(rel_x{veh} < safety_set.g.min) || any(rel_x{veh} > safety_set.g.max)
      safe = true;
    else
      safety_vals{veh} = eval_u(safety_set.g, safety_set.data, rel_x{veh});
      if safety_vals{veh} > small
        safe = true;
      else
        safe = false;
        avoidance_started = true;
        evader_ind = veh;
      end
    end
  end
  
  % This should only be performed once
  if safe
    for veh = 1:2
      u{veh} = [planes{veh}.vrange(1); 0];
    end
    return
  else
    u{other(evader_ind)} = [planes{other(evader_ind)}.vrange(1); 0];
    
    p = eval_u(safety_set.g, safety_set.deriv, rel_x{evader_ind});
    u{evader_ind} = planeRel.optCtrl([], rel_x{evader_ind}, p, 'max');    
  end  
end
end

function xOut = rotate_to_goal_sat_frame(xIn, init_heading)

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

function dist = dist_pt_to_line(a, p, n)

dist = norm( a-p - sum((a-p).*n)*n );
end