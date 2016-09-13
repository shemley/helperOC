classdef Quad6D < DynSys
  % Note: Since quad is a "handle class", we can pass on
  % handles/pointers to other plane objects
  % e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
  % Also see constructor
  
  properties
    % control bounds
    T1Max
    T1Min
    T2Max
    T2Min
    
    m % mass
    
    grav %gravity
    
    transDrag %translational drag
    
    rotDrag %rotational drag

    Iyy %moment of Inertia
    
    l %length from center of mass to end
    
    dims %dimensions that are active
    
  end
  
  methods
    function obj = Quad6D(x, T1Max, T1Min, T2Max, T2Min,...
        m, grav, transDrag, rotDrag, Iyy, l, dims)
      % obj = Quad6D(x, T1Max, T1Min, T2Max, T2Min,...
      %  m, grav, transDrag, rotDrag, Iyy, l)
      %
      % Constructor. Creates a quadcopter object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    insert these later
      %
      % Inputs:
      %   x      - state: [xpos; xvel; ypos; yvel; psipos; psivel]
      %   T1Max, T1Min, T2Max, T2Min - inputs
      %   m - mass
      %   grav - gravity
      %   transDrag - translational Drag
      %   rotDrag - rotaitonal Drag
      %   Iyy - moment of inertia
      %   l - ???
      %
      % Output:
      %   obj       - a Quadcopter Object
      %
      % Sylvia Herbert 2016-9-12
      
      if nargin < 2
        T1Max = 5;
      end
      
      if nargin < 3
        T1Min = -T1Max;
      end
      
      if nargin < 4
        T2Max = 5;
      end
      
      if nargin < 5
        T2Min = -T2Max;
      end
      
      if nargin < 6
        m = 3;
      end
      
      if nargin < 7
        grav = 9.81;
      end
      
      if nargin < 8
        transDrag = 5;
      end
      
      if nargin < 9
        rotDrag = 5;
      end
      
      if nargin < 10
        Iyy = 5;
      end
      
      if nargin < 11
        l = 5;
      end
      
      if nargin <12
        dims = [1 2 5 6];
      end
      
      % Basic vehicle properties
      obj.pdim = [1 3]; % Position dimensions
      obj.hdim = 5;   % Heading dimensions
      obj.nx = length(dims);
      obj.nu = 2;  
      
      obj.x = x;
      obj.xhist = obj.x;
      obj.dims = dims;
      
      obj.T1Max = T1Max;
      obj.T1Min = T1Min;
      obj.T2Max = T2Max;
      obj.T2Min = T2Min;
      obj.m = m;
      obj.grav = grav;
      obj.transDrag = transDrag;
      obj.rotDrag = rotDrag;
      obj.Iyy = Iyy;
      obj.l = l;
    end
    
  end % end methods
end % end classdef
