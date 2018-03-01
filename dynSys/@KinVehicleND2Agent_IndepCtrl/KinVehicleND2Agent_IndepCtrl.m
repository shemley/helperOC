classdef KinVehicleND2Agent_IndepCtrl < DynSys
  properties
    uMax
    dMax
  end
  
  methods
    function obj = KinVehicleND2Agent_IndepCtrl(x, uMax, dMax)
      % obj = KinVehicleND(x, vMax)
      %
      % Dynamics: (2D example)
      %    Agent 1:
      %    \dot{x}_1 = u_x
      %    \dot{x}_2 = u_y
      %         u_x <= uMax_1
      %         u_y <= uMax_2
      %    Agent 2:
      %    \dot{x}_3 = d_x
      %    \dot{x}_4 = d_y
      %         d_x <= dMax_1
      %         d_y <= dMax_2
      
      %% State could be of any number of dimensions
      if ~iscolumn(x)
        x = x';
      end
      if ~iscolumn(uMax)
        uMax = uMax';
      end
      if ~iscolumn(dMax)
        dMax = dMax';
      end  
      
      if rem(length(x),2) ~= 0
        error('length(x) must be even!')
      end
      
      obj.nx = length(x);
      obj.nu = obj.nx/2;
      obj.nd = obj.nu;      
      
      %% Velocity
      if nargin < 2
        uMax = ones(obj.nu,1);
        dMax = ones(obj.nd,1);      
      elseif nargin < 3
        dMax = ones(obj.nd,1);
      end
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMax = uMax;
      obj.dMax = dMax;
    end
    
  end % end methods
end % end classdef
