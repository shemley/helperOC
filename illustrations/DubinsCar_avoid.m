function DubinsCar_avoid(which_illustration, save_png)

if nargin < 2
  save_png = true;
end

if save_png
  folder = sprintf('%s_%s', mfilename, which_illustration);
  
  if ~exist(folder, 'dir')
    system(sprintf('mkdir %s', folder));
  end
end

%% Dynamical system
wMax = 0.5;
speed = 2;
dMax = [0; 0; 0];


%% Grid
gN = 101;
gMin = [-5; -5; -pi];
gMax = [5; 5; pi];
g = createGrid(gMin, gMax, gN, 3);

%% Time
tMax = 2;
dt = 0.1;
tau = 0:dt:tMax;
colors = flip(jet(length(tau)));

%% Collision set
R = 2;
data0 = shapeCylinder(g, 3, [0;0;0], R);

switch which_illustration
  case 'Trajectories'
    datafile = sprintf('%s_data.mat', mfilename);
    load(datafile)    
    
    f = figure;
    f.Color = 'white';
    plotDisk([0;0], R, 'color', colors(1,:), 'LineWidth', 3);
    xlim([-4.5 2.5])
    ylim([-3.5 3.5])
    axis square
    hold on

    x0 = [-3.5, 0.5, 0];
    u = [-wMax; 0.1*wMax];
    maxTaui = ceil(0.7*length(tau));
    simulateTrajectories(x0, maxTaui, u, deriv, wMax, speed, dMax, dt, g, ...
      'b', save_png, folder, 'unsafe')
    
    x0 = [-4, -1, 0];
    u = [wMax; -0.2*wMax];    
    simulateTrajectories(x0, maxTaui, u, deriv, wMax, speed, dMax, dt, g, ...
      [0 0.5 0], save_png, folder, 'safe')
    
    x0 = [-2, -1, 0];
    u = [];
    maxTaui = ceil(0.9*length(tau));
    simulateTrajectories(x0, maxTaui, u, deriv, wMax, speed, dMax, dt, g, ...
      [1 0.5 0], save_png, folder, 'through')
    
  case 'BRS_computation'
    datafile = sprintf('%s_data.mat', mfilename);
    
    if exist(datafile, 'file')
      load(datafile)
    else
      %% Solver parameters
      dynSys = DubinsCar([0;0;0], wMax, speed, dMax);
      params.dynSys = dynSys;
      params.grid = g;
      params.uMode = 'max';
      params.dMode = 'min';
      extraArgs.visualize = true;
      
      %% Call solver
      data = HJIPDE_solve(data0, tau, params, 'zero', extraArgs);
      
      %% Compute gradient
      deriv = cell(length(tau),1);
      for i = 1:length(tau)
        deriv{i} = computeGradients(g, data(:,:,:,i));
      end
      save(datafile, 'g', 'data', 'tau', 'deriv', 'dynSys', '-v7.3')
    end
    
    %% Visualize first few sets on the same plot
    first_few = 3;
    f = figure;
    f.Color = 'white';
    inds = [1 3 7];
    
    hs_ff = cell(first_few,1);
    hl_ff = cell(first_few,1);
    
    for i = inds
      [g2D, data2D] = proj(g, data, [0 0 1], dynSys.x(3));
      
      hs_ff{i} = surf(g2D.vs{1}, g2D.vs{2}, data2D(:,:,i)', ...
        'LineStyle', 'none', 'FaceColor', colors(i,:), 'FaceAlpha', 0.5);
      hold on
      
      hl_ff{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));
      hl_ff{i}.LineWidth = 4;
      
      if i == 1
        camlight left
        camlight right
        
        xlim([-4 2.25])
        ylim([-3 3])
        zlim([-2 1.5])
        xlabel('x', 'FontSize', 14)
        ylabel('y', 'FontSize', 14)
        zlabel('V(x, y, t)', 'FontSize', 14)
        grid on
      end
      
      if save_png
        view(2)
        drawnow
        
        for j = 1:i
          hs_ff{j}.Visible = 'off';
        end
        fig2Dff_filename = sprintf('%s/fig2Dff_%d', folder, i);
        export_fig(fig2Dff_filename, '-png', '-m2')
      end
      
      view(3)
      drawnow
      
      if save_png
        for j = 1:i
          hs_ff{j}.Visible = 'on';
        end
        fig3Dff_filename = sprintf('%s/fig3Dff_%d', folder, i);
        export_fig(fig3Dff_filename, '-png', '-m2')
      end
    end
    
    %% Visualize in time
    f = figure;
    f.Color = 'white';
    
    for i = 1:length(tau)
      [g2D, data2D] = proj(g, data, [0 0 1], dynSys.x(3));
      
      if i == 1
        hs = surf(g2D.vs{1}, g2D.vs{2}, data2D(:,:,i)');
        hs.LineStyle = 'none';
        hs.FaceColor = colors(i,:);
        hs.FaceAlpha = 0.5;
        hold on
        
        hl = visSetIm(g2D, data2D(:,:,i), colors(i,:));
        hl.LineWidth = 4;
        
        camlight left
        camlight right
        
        xlabel('x', 'FontSize', 14)
        ylabel('y', 'FontSize', 14)
        zlabel('V(x, y, t)', 'FontSize', 14)
        grid on
      else
        hs.ZData = data2D(:,:,i)';
        hs.FaceColor = colors(i,:);
        
        hl.ZData = data2D(:,:,i);
        hl.Color = colors(i,:);
        
      end
      
      if save_png
        view(2)
        drawnow
        
        fig2D_filename = sprintf('%s/fig2D_%d', folder, i);
        export_fig(fig2D_filename, '-png', '-m2')
        
      end
      
      view(3)
      drawnow
      
      if save_png
        fig3D_filename = sprintf('%s/fig3D_%d', folder, i);
        export_fig(fig3D_filename, '-png', '-m2')
      end
    end
    
  otherwise
    error('Unknown illustration!')
end

end

function simulateTrajectories(x0, maxi, u, deriv, wMax, speed, dMax, dt, g, ...
  c, save_png, folder, name)

plot(x0(1), x0(2), 's', 'color', c, 'MarkerFaceColor', c);
drawnow

if save_png
  fig_name = sprintf('%s/%s_0', folder, name);
  export_fig(fig_name, '-png', '-m2')
end

dCars = cell(length(u)+1, 1);

extraArgs.arrowLength = 0;
extraArgs.LineStyle = '-';
extraArgs.Color = c;
extraArgs.LineWidth = 2;
extraArgs.MarkerSize = 25;

% Create copies of Dubins Car
for i = 1:length(dCars)
  dCars{i} = DubinsCar(x0, wMax, speed, dMax);
  
  if i == length(dCars)
    extraArgs.LineWidth = 5;
  end
  
  for ti = 2:maxi
    if i == length(dCars)
      p = eval_u(g, deriv{end}, dCars{i}.x);
      uThis = dCars{i}.optCtrl([], [], p, 'max');      
    else
      uThis = u(i);
    end
    dCars{i}.updateState(uThis, dt);
    dCars{i}.plotPosition(extraArgs);

    drawnow
    if save_png
      fig_name = sprintf('%s/%s_%d_%d', folder, name, i, ti);
      export_fig(fig_name, '-png', '-m2')
    end    
  end
  
end
end
