function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
% Agent 2 is the disturbance
%     Dynamics of the 2 agent system (2D)
%         \dot{x}_1 = u_1
%         \dot{x}_2 = u_2
%         \dot{x}_3 = d_1
%         \dot{x}_4 = d_2

%% Input processing
if nargin < 5
  dMode = 'max';
end

if strcmp(dMode, 'max')
  s = 1;
elseif strcmp(dMode, 'min')
  s = -1;
else
  error('Unknown dMode!')
end

%% Optimal control
dOpt = deriv((obj.nu+1):end);
if iscell(deriv)
  for i = 1:obj.nd
    dOpt{i} = s*obj.dMax(i)*sign(dOpt{i});
  end

else  
  for i = 1:obj.nd
    dOpt(i) = s*obj.dMax(i)*sign(dOpt(i));
  end
end

end