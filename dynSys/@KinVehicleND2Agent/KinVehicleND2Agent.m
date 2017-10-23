classdef KinVehicleND2Agent < DynSys
  properties
    uMax
    dMax
  end
  
  methods
    function obj = KinVehicleND2Agent(x, uMax, dMax)
      % obj = KinVehicleND(x, vMax)
      %
      % Dynamics: (2D example)
      %    Agent 1:
      %    \dot{x}_1 = u_x
      %    \dot{x}_2 = u_y
      %         u_x^2 + u_y^2 <= uMax^2
      %    Agent 2:
      %    \dot{x}_3 = d_x
      %    \dot{x}_4 = d_y
      %         d_x^2 + d_y^2 <= dMax^2
      
      %% State could be of any number of dimensions
      if ~iscolumn(x)
        x = x';
      end
      
      if rem(length(x),2) ~= 0
        error('length(x) must be even!')
      end
      
      obj.nx = length(x);
      obj.nu = obj.nx/2;
      obj.nd = obj.nu;      
      
      %% Velocity
      if nargin < 2
        uMax = 1;
        dMax = 1;      
      elseif nargin < 3
        dMax = 1;
      end
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMax = uMax;
      obj.dMax = dMax;
    end
    
  end % end methods
end % end classdef
