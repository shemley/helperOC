function uOpt = optCtrl(obj, t, y, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, dims)
% Agent 1 is the control
%     Dynamics of the 2 agent system (2D)
%         \dot{x}_1 = u_1
%         \dot{x}_2 = u_2
%         \dot{x}_3 = d_1
%         \dot{x}_4 = d_2

%% Input processing
if nargin < 5
  uMode = 'min';
end

if strcmp(uMode, 'max')
  s = 1;
elseif strcmp(uMode, 'min')
  s = -1;
else
  error('Unknown uMode!')
end

%% Optimal control
uOpt = deriv(1:obj.nu);
denom = 0;
if iscell(deriv)
  for i = 1:obj.nu
    denom = denom + deriv{i}.^2;
  end
  denom = sqrt(denom);

  for i = 1:obj.nu
    uOpt{i} = s*obj.uMax*uOpt{i} ./ denom;
    uOpt{i}(denom == 0) = 0;
  end

else
  for i = 1:obj.nu
    denom = denom + deriv(i).^2;
  end
  denom = sqrt(denom);
  
  if denom > 0
  for i = 1:obj.nu
    uOpt(i) = s*obj.uMax*uOpt(i) ./ denom;
  end
  else
    uOpt = zeros(obj.nu, 1);
  end
end



end