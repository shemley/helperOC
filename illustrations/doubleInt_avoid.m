function doubleInt_avoid(which_illustration, save_png)

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

x = [2; 0];
urange = [-1 1];


%% Grid
gN = 101;
gMin = [0; -2];
gMax = [4; 2];
g = createGrid(gMin, gMax, gN);

data0 = -shapeRectangleByCorners(g, [0.5 -1.5], [3.5 1.5]);

%% Time
tMax = 2;
dt = 0.1;
tau = 0:dt:tMax;
colors = flip(jet(length(tau)));

switch which_illustration
  case 'Trajectories'
  case 'BRS_computation'
    datafile = sprintf('%s_data.mat', mfilename);
    
    if exist(datafile, 'file')
      load(datafile)
    else
      %% Solver parameters
      dynSys = DoubleInt(x, urange);
      params.dynSys = dynSys;
      params.grid = g;
      params.uMode = 'max';
      extraArgs.visualize = true;
      
      %% Call solver
      data = HJIPDE_solve(data0, tau, params, 'zero', extraArgs);
      
      %% Compute gradient
      deriv = cell(length(tau),1);
      for i = 1:length(tau)
        deriv{i} = computeGradients(g, data(:,:,i));
      end
      
      BRS.g = g;
      BRS.data = data;
      BRS.tau = tau;
      BRS.deriv = deriv;
      save(datafile, 'BRS', 'dynSys', '-v7.3')
    end
    
    %% Visualize first and final set
    f = figure;
    f.Color = 'white';
    inds = [1 length(BRS.tau)];
    
    hl1_ff = cell(2,1);
    
    for i = inds
      hl1_ff{i} = visSetIm(BRS.g, BRS.data(:,:,i), colors(i,:));
      hl1_ff{i}.LineWidth = 4;
      hold on
      
      if i == 1
        axis square
        xlabel('Position', 'FontSize', 16)
        ylabel('Velocity', 'FontSize', 16)
      end
      
      drawnow
      
      if save_png
        basic_fig_filename = sprintf('%s/basic_fig_%d', folder, i);
        export_fig(basic_fig_filename, '-png', '-m2')
      end
      
    end

  otherwise
    error('Unknown illustration!')
end
end