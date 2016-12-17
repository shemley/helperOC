function DubinsCar_avoid(save_png)

if nargin < 1
  save_png = true;
end

if save_png
  folder = mfilename;
  
  if ~exist(folder, 'dir')
    system(sprintf('mkdir %s', folder));
  end
end

datafile = sprintf('%s_data.mat', mfilename);

if exist(datafile, 'file')
  load(datafile)
else
  x0 = [-5; 0; 0];
  wMax = 0.5;
  speed = 2;
  dMax = [0; 0; 0];
  
  dynSys = DubinsCar(x0, wMax, speed, dMax);
  
  %% Grid
  gN = 101;
  gMin = [-5; -5; -pi];
  gMax = [5; 5; pi];
  g = createGrid(gMin, gMax, gN, 3);
  
  %% Time
  tMax = 2;
  dt = 0.1;
  tau = 0:dt:tMax;
  
  %% Collision set
  data0 = shapeCylinder(g, 3, [0;0;0], 2);
  
  %% Solver parameters
  params.dynSys = dynSys;
  params.grid = g;
  params.uMode = 'max';
  params.dMode = 'min';
  extraArgs.visualize = true;
  
  %% Call solver
  data = HJIPDE_solve(data0, tau, params, 'zero', extraArgs);
  save(datafile, 'g', 'data', 'tau', 'dynSys')
end

%% Visualize first few sets on the same plot
first_few = 3;
f = figure;
f.Color = 'white';
colors = lines(first_few);

hs_ff = cell(first_few,1);
hl_ff = cell(first_few,1);

for i = 1:first_few
  [g2D, data2D] = proj(g, data, [0 0 1], dynSys.x(3));
  
    hs_ff{i} = surf(g2D.vs{1}, g2D.vs{2}, data2D(:,:,i)', ...
      'LineStyle', 'none', 'FaceColor', colors(i,:), 'FaceAlpha', 0.5);
    hold on
    
    hl_ff{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));
    hl_ff{i}.LineWidth = 4;  
  
  if i == 1
    camlight left
    camlight right
    
    xlim([-2.5 2.5])
    ylim([-2.5 2.5])
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
colors = flip(jet(length(tau)));

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