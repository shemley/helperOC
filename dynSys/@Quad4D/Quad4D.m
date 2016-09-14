classdef Quad4D < DynSys
  properties
    uMin    % Control bounds
    uMax
  end % end properties

  methods
    function obj = Quad4D(x, uMin, uMax)
      % obj = Quad4D(x, uMax)
      %
      % Constructor. Creates a quadrotor object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{p}_x = v_x
      %    \dot{v}_x = u_x
      %    \dot{p}_y = v_y
      %    \dot{v}_y = u_y
      %       uMin <= u_x <= uMax
      %
      % Inputs:   x   - state: [xpos; xvel; ypos; yvel]
      % Output:   obj - a quadrotor object
      
      if nargin < 2
        uMax = 3;
        uMin = -3;
      end
      
      % Make sure initial state is 4D
      if numel(x) ~= 4
        error('Quadrotor state must be 4D.')
      end

      % Make sure initial state is a column vector
      if ~iscolumn(x)
        x = x';
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMax = uMax;
      obj.uMin = uMin;
      
      obj.pdim = [1 3];
      obj.vdim = [2 4];
    end % end constructor
  end % end methods
end % end class