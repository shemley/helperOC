function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the DubinsCar
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% Optimal control
if strcmp(dMode, 'max')
  if any(obj.dims == 1)
    dOpt{1} = (deriv{obj.dims==1}>=0)*obj.dMax(1) + ...
      (deriv{obj.dims==1}<0)*(-obj.dMax(1));
  end
  
  if any(obj.dims == 2)
    dOpt{2} = (deriv{obj.dims==2}>=0)*obj.dMax(2) + ...
      (deriv{obj.dims==2}<0)*(-obj.dMax(2));
  end

elseif strcmp(dMode, 'min')
  if any(obj.dims == 1)
    dOpt{1} = (deriv{obj.dims==1}>=0)*(-obj.dMax(1)) + ...
      (deriv{obj.dims==1}<0)*obj.dMax(1);
  end
  
  if any(obj.dims == 2)
    dOpt{2} = (deriv{obj.dims==2}>=0)*(-obj.dMax(2)) + ...
      (deriv{obj.dims==2}<0)*obj.dMax(2);
  end
else
  error('Unknown dMode!')
end

end