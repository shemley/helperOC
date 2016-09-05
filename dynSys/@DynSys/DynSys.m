classdef DynSys < handle
  % Dynamical Systems class
  %   Subclasses: quadrotor, Dubins vehicle (under construction)
  
  properties
    nx          % Number of state dimensions
    nu          % Number of control inputs
    nd          % Number of disturbance dimensions
    
    x           % State
    u           % Recent control signal
    
    xhist       % History of state
    uhist       % History of control
    
    pdim        % position dimensions
    vdim        % velocity dimensions
    hdim        % heading dimensions
    
    %% Figure handles
    hpxpy           % Position
    hpxpyhist       % Position history
    hvxvy           % Velocity
    hvxvyhist       % Velocity history
    
    % Position velocity (so far only used in DoubleInt)
    hpv = cell(2,1);
    hpvhist = cell(2,1);
    
    % Data (any data that you may want to store for convenience)
    data
  end % end properties

  % No constructor in DynSys class. Use constructors in the subclasses
  
end % end class