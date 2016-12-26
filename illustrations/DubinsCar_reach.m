function DubinsCar_reach(which_illustration, save_png)

if nargin < 2
  save_png = true;
end

if save_png
  folder = sprintf('%s_%s', mfilename, which_illustration);
  
  if ~exist(folder, 'dir')
    system(sprintf('mkdir %s', folder));
  end
else
  folder = [];
end

%% Dynamical system
wMax = 0.5;
speed = 2;
dMax = [0; 0; 0];

%% Grid
gN = 101;
gMin = [-7.5; -7.5; -pi];
gMax = [7.5; 7.5; pi];
g = createGrid(gMin, gMax, gN, 3);

fig_size = [1150 450];

%% Time
tMax = 2;
dt = 0.1;
tau = 0:dt:tMax;
colors = flip(jet(length(tau)));

%% Collision set
R = 2;
data0 = shapeCylinder(g, 3, [0;0;0], R);

switch which_illustration
  case 'BRS_computation'
    datafile = sprintf('%s_data.mat', mfilename);
    
    if exist(datafile, 'file')
      load(datafile)
    else
      %% Solver parameters
      dynSys = DubinsCar([0;0;0], wMax, speed, dMax);
      params.dynSys = dynSys;
      params.grid = g;
      params.uMode = 'min';
      extraArgs.visualize = true;
      
      %% Call solver
      data = HJIPDE_solve(data0, tau, params, 'zero', extraArgs);
      
      %% Compute gradient
      BRS.g = g;
      BRS.data = data;
      BRS.tau = tau;
      save(datafile, 'BRS', 'dynSys', '-v7.3')
    end
    
    %% Visualize first and final set
    f = figure;
    f.Color = 'white';
    inds = [1 length(BRS.tau)];
    
    hl1_ff = cell(2,1);
    
    for i = inds
      [g2D, data2D] = proj(BRS.g, BRS.data, [0 0 1], dynSys.x(3));
      hl1_ff{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));      

      hl1_ff{i}.LineWidth = 4;
      hold on
      
      if i == 1
        axis square
        xlabel('Position', 'FontSize', 16)
        ylabel('Velocity', 'FontSize', 16)
      end
      
      drawnow
      
    end
    
    if save_png
      basic_fig_filename = sprintf('%s/basic_fig_%d', folder, i);
      export_fig(basic_fig_filename, '-png', '-m2')
    end
    
  otherwise
    error('Unknown illustration!')
end
end