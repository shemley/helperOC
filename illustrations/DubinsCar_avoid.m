function DubinsCar_avoid(which_illustration, save_png)

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
gMin = [-5; -5; -pi];
gMax = [5; 5; pi];
g = createGrid(gMin, gMax, gN, 3);

view_angle = [-48 11];
fig_size = [1150 450];

%% Time
tMax = 2;
dt = 0.1;
tau = 0:dt:tMax;
colors = flip(jet(length(tau)));

%% Collision set
R = 2;
data0 = shapeCylinder(g, 3, [0;0;0], R);
% data0 = shapeRectangleByCenter(g, [0;0;0], [R/2; R/2; inf]);

switch which_illustration
  case 'Trajectories'
    datafile = sprintf('%s_data.mat', mfilename);
    load(datafile)
    
    f = figure;
    f.Position(3:4) = fig_size;
    f.Color = 'white';

    [g2D, data02D] = proj(g, data0, [0 0 1]);
    
    hs1 = subplot(1,2,1);
    
    plotDisk([0;0], R, 'color', colors(1,:), 'LineWidth', 3);
    hold on
    xlim([-4 4])
    ylim([-4 4])
    axis square
%     xlabel('x', 'FontSize', 16)
%     ylabel('y', 'FontSize', 16)
    grid on
        
    hs1.FontSize = 16;
    
    hs2 = subplot(1,2,2);
    hvf = surf(g2D.vs{1}, g2D.vs{2}, data02D');
    hvf.FaceAlpha = 0.5;
    hvf.FaceColor = colors(1,:);
    hvf.LineStyle = 'none';
    hold on
    
    camlight left
    camlight right
    
    plotDisk([0;0], R, 'color', colors(1,:), 'LineWidth', 3);
    
    xlim([-4 4])
    ylim([-4 4])
    zlim([-2 4])
    axis square
    view(view_angle)
%     
%     xlabel('x', 'FontSize', 16)
%     ylabel('y', 'FontSize', 16)
%     zlabel('l(x, y)', 'FontSize', 16)
    grid on
    box on
    hs2.FontSize = 16;
    
    if save_png
      export_fig(sprintf('%s/traj_basic', folder), '-png', '-m2', '-nocrop');
    end
    
    dynSys = DubinsCar([0;0;0], wMax, speed, dMax);
    
    x0 = [-3.5, 0.5, 0];
    u = [-wMax; 0.1*wMax];
    maxTaui = ceil(0.7*length(tau));
    
    simulateTrajectories(x0, maxTaui, u, BRS, dynSys, dt, 'b', save_png, ...
      folder, 'unsafe')
    
    x0 = [-4, -1, 0];
    u = [];
    simulateTrajectories(x0, maxTaui, u, BRS, dynSys, dt, [0 0.5 0], ...
      save_png, folder, 'safe')
    
%     x0 = [-2, -1, 0];
%     u = [];
%     maxTaui = ceil(0.9*length(tau));
%     simulateTrajectories(x0, maxTaui, u, BRS, dynSys, dt, [1 0.5 0], ...
%       save_png, folder, 'through')
    
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
      
      BRS.g = g;
      BRS.data = data;
      BRS.tau = tau;
      BRS.deriv = deriv;
      save(datafile, 'BRS', 'dynSys', '-v7.3')
    end
    
    %% Visualize first and final set
    f = figure;
    f.Color = 'white';
    hs = subplot(1,1,1);
    inds = [1 length(BRS.tau)];
    
    hl1_ff = cell(2,1);
    
    for i = inds
      [g2D, data2D] = proj(BRS.g, BRS.data, [0 0 1], dynSys.x(3));
      
      
      hl1_ff{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));
      hl1_ff{i}.LineWidth = 4;
      hold on
      
      if i == 1
        axis square
        hs.FontSize = 16;
%         xlabel('x', 'FontSize', 16)
%         ylabel('y', 'FontSize', 16)
      end
      
      drawnow
    end
    
    if save_png
      basic_fig_filename = sprintf('%s/basic_fig', folder);
      export_fig(basic_fig_filename, '-png', '-m2', '-nocrop')
    end
    
    %% Visualize first few sets on the same plot
    first_few = 3;
    f = figure;
    f.Color = 'white';
    f.Position(3:4) = fig_size;
    inds = [1 3 5];
    
    hs_ff = cell(first_few,1);
    hl_ff = cell(first_few,1);
    hl_ff1 = cell(first_few,1);
    
    for i = inds
      [g2D, data2D] = proj(BRS.g, BRS.data, [0 0 1], dynSys.x(3));
      
      % 2D line plot only
      hs1 = subplot(1,2,1);
      hl_ff1{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));
      hl_ff1{i}.LineWidth = 4;
      
      xlim([-4 4])
      ylim([-4 4])
      axis square
%       xlabel('x', 'FontSize', 16)
%       ylabel('y', 'FontSize', 16)
      box on
      grid on
      hold on
      
      % 3D surface plot and line plot
      hs2 = subplot(1,2,2);
      hs_ff{i} = surf(g2D.vs{1}, g2D.vs{2}, data2D(:,:,i)', ...
        'LineStyle', 'none', 'FaceColor', colors(i,:), 'FaceAlpha', 0.5);
      hold on
      
      hl_ff{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:));
      hl_ff{i}.LineWidth = 4;
      
      if i == 1
        camlight left
        camlight right
        
        xlim([-4 4])
        ylim([-4 4])
        zlim([-2 4])
        axis square
        view(view_angle)
        hs1.FontSize = 16;
        hs2.FontSize = 16;
%         xlabel('x', 'FontSize', 16)
%         ylabel('y', 'FontSize', 16)
%         zlabel('V(x, y, t)', 'FontSize', 16)
        box on
        grid on
      end

      if save_png
        figff_filename = sprintf('%s/figff_%d', folder, i);
        export_fig(figff_filename, '-png', '-m2', '-nocrop')
      end
    end
    
    %% Visualize in time
    f = figure;
    f.Color = 'white';
    f.Position(3:4) = fig_size;
    
    for i = 1:length(tau)
      [g2D, data2D] = proj(BRS.g, BRS.data, [0 0 1], dynSys.x(3));
      
      if i == 1
        % 2D plot
        hs3 = subplot(1,2,1);
        hl1 = visSetIm(g2D, data2D(:,:,i), colors(i,:));
        hl1.LineWidth = 4;   
        hold on
        
%         xlabel('x', 'FontSize', 16)
%         ylabel('y', 'FontSize', 16)
        axis square
        box on
        grid on        
        
        hs3.FontSize = 16;
        
        % 3D plot
        hs4 = subplot(1,2,2);
        hs = surf(g2D.vs{1}, g2D.vs{2}, data2D(:,:,i)');
        hs.LineStyle = 'none';
        hs.FaceColor = colors(i,:);
        hs.FaceAlpha = 0.5;
        hold on
        
        hl = visSetIm(g2D, data2D(:,:,i), colors(i,:));
        hl.LineWidth = 4;
        
        camlight left
        camlight right
        
%         xlabel('x', 'FontSize', 16)
%         ylabel('y', 'FontSize', 16)
%         zlabel('V(x, y, t)', 'FontSize', 16)
        axis square
        view(view_angle)
        box on
        grid on
        
        hs4.FontSize = 16;
        
      else
        hl1.ZData = data2D(:,:,i);
        hl1.Color = colors(i,:);
        
        hs.ZData = data2D(:,:,i)';
        hs.FaceColor = colors(i,:);
        
        hl.ZData = data2D(:,:,i);
        hl.Color = colors(i,:);
        
      end
      
      if save_png
        fig3D_filename = sprintf('%s/fig_%d', folder, i);
        export_fig(fig3D_filename, '-png', '-m2', '-nocrop')
      end
    end
    
  otherwise
    error('Unknown illustration!')
end

end

function simulateTrajectories(x0, maxi, u, BRS, dynSys, dt, c, save_png, ...
  folder, name)

[g2D, data02D] = proj(BRS.g, BRS.data(:,:,:,1), [0 0 1]);

z0 = eval_u(g2D, data02D, x0(1:2));

subplot(1,2,1)
plot(x0(1), x0(2), 's', 'color', c, 'MarkerFaceColor', c);

subplot(1,2,2)
plot3(x0(1), x0(2), z0, 's', 'color', c, 'MarkerFaceColor', c);
drawnow

if save_png
  fig_name = sprintf('%s/%s_0', folder, name);
  export_fig(fig_name, '-png', '-m2', '-nocrop')
end

dCars = cell(length(u)+1, 1);

% Create copies of Dubins Car
for i = 1:length(dCars)
  dCars{i} = DubinsCar(x0, dynSys.wMax, dynSys.speed, dynSys.dMax);
  
  z = z0;
  zhist = z;
  
  if i == length(dCars)
    LineWidth = 5;
  else
    LineWidth = 2;
  end
  
  for ti = 2:maxi
    if i == length(dCars)
      p = eval_u(BRS.g, BRS.deriv{end}, dCars{i}.x);
      uThis = dCars{i}.optCtrl([], [], p, 'max');
    else
      uThis = u(i);
    end
    dCars{i}.updateState(uThis, dt);
    
    z = eval_u(g2D, data02D, dCars{i}.x(1:2));
    zhist = [zhist z];
    qu = 0.5*cos(dCars{i}.x(3));
    qv = 0.5*sin(dCars{i}.x(3));
    
    if ti == 2
      subplot(1,2,1)
      hq1 = quiver(dCars{i}.x(1), dCars{i}.x(2), qu, qv, 'color', c, ...
        'markersize', 25, 'AutoScaleFactor', 0, 'Marker', '.');
      ht1 = plot(dCars{i}.xhist(1,:), dCars{i}.xhist(2,:), 'color', ...
        c, 'linewidth', LineWidth, 'linestyle', '-');
      
      subplot(1,2,2)
      hq = quiver3(dCars{i}.x(1), dCars{i}.x(2), z, qu, qv, 0, 'color', c, ...
        'markersize', 25, 'AutoScaleFactor', 0, 'Marker', '.');
      ht = plot3(dCars{i}.xhist(1,:), dCars{i}.xhist(2,:), zhist, 'color', ...
        c, 'linewidth', LineWidth, 'linestyle', '-');
    else
      hq1.XData = dCars{i}.x(1);
      hq1.YData = dCars{i}.x(2);
      hq1.UData = qu;
      hq1.VData = qv;
      
      ht1.XData = dCars{i}.xhist(1,:);
      ht1.YData = dCars{i}.xhist(2,:);      
      
      hq.XData = dCars{i}.x(1);
      hq.YData = dCars{i}.x(2);
      hq.ZData = z;
      hq.UData = qu;
      hq.VData = qv;
      
      ht.XData = dCars{i}.xhist(1,:);
      ht.YData = dCars{i}.xhist(2,:);
      ht.ZData = zhist;
    end
    
    drawnow
    if save_png
      fig_name = sprintf('%s/%s_%d_%d', folder, name, i, ti);
      export_fig(fig_name, '-png', '-m2', '-nocrop')
    end
  end
  
end
end
