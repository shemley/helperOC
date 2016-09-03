classdef Quad10D < DynSys
  properties
    uMin        % Control bounds (3x1 vector)
    uMax
    
    % Constants
    %   The choices of n0, d1, d0 actually results in a very large
    %   steady state error in the pitch/roll; this seems to be
    %   expected according to Pat's report
    n0 = 10     % Angular dynamics parameters
    d1 = 8
    d0 = 10
    
    kT = 0.91   % Thrust coefficient (vertical direction)
    g = 9.81    % Acceleration due to gravity (for convenience)
    m = 1.3     % Mass
  end
  
  methods
    function obj = Quad10D(x, uMin, uMax)
      % obj = Quad10D(x, uMin, uMax)
      %     Constructor for a 10D quadrotor
      %
      % Dynamics:
      %     \dot x_1 = x_2
      %     \dot x_2 = g * tan(x_3)
      %     \dot x_3 = -d1 * x_3 + x_4
      %     \dot x_4 = -d0 x_3 + n0 * u1
      %     \dot x_5 = x_6
      %     \dot x_6 = g * tan(x_7)
      %     \dot x_7 = -d1 * x_7 + x_8
      %     \dot x_8 = -d0 x_7 + n0 * u2
      %     \dot x_9 = x_10
      %     \dot x_10 = kT * u3
      %         uMin <= [u1; u2; u3] <= uMax
      
      obj.nx = 10;
      obj.nu = 3;
      obj.pdim = [1 5 9];
      obj.vdim = [2 6 10];
      
      if nargin < 1
        x = zeros(obj.nx, 1);
      end
      
      if nargin < 2
        uMax = [10/180*pi; 10/180*pi; 2*obj.g];
        uMin = [-10/180*pi; -10/180*pi; 0];
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMax = uMax;
      obj.uMin = uMin;
    end
  end
end