function uOpt = optCtrl(obj, ~, ~, deriv, uMode, ~)
% uOpt = optCtrl(obj, ~, ~, deriv, uMode, ~)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
uOpt = cell(obj.nu, 1);
if strcmp(uMode, 'max')
  uOpt{1} = (deriv{2}>=0)*obj.uMax + (deriv{2}<0)*obj.uMin;
  uOpt{2} = (deriv{4}>=0)*obj.uMax + (deriv{4}<0)*obj.uMin;
elseif strcmp(uMode, 'min')
  uOpt{1} = (deriv{2}>=0)*obj.uMin + (deriv{2}<0)*obj.uMax;
  uOpt{2} = (deriv{4}>=0)*obj.uMin + (deriv{4}<0)*obj.uMax;
else
  error('Unknown uMode!')
end

end