function dOpt = optDstb(obj, ~, ~, deriv, dMode, dims)
% uOpt = optCtrl(obj, t, y, deriv, uMode, dims)
%     Dynamics of the DubinsCar
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u_1 = u_1

%% Input processing
if nargin < 5
  dMode = 'max';
end

if nargin < 6
  dims = 1:obj.nx;
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% Optimal control
if strcmp(dMode, 'max')
  if any(dims == 1)
    dOpt{1} = (deriv{dims==1}>=0)*obj.dMax(1) + ...
      (deriv{dims==1}<0)*(-obj.dMax(1));
  end
  
  if any(dims == 2)
    dOpt{2} = (deriv{dims==2}>=0)*obj.dMax(2) + ...
      (deriv{dims==2}<0)*(-obj.dMax(2));
  end

elseif strcmp(dMode, 'min')
  if any(dims == 1)
    dOpt{1} = (deriv{dims==1}>=0)*(-obj.dMax(1)) + ...
      (deriv{dims==1}<0)*obj.dMax(1);
  end
  
  if any(dims == 2)
    dOpt{2} = (deriv{dims==2}>=0)*(-obj.dMax(2)) + ...
      (deriv{dims==2}<0)*obj.dMax(2);
  end
else
  error('Unknown dMode!')
end

end