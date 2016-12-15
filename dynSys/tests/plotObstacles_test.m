function plotObstacles_test()

obsBRS_filenames = 'Quad4D_test.mat';

global_gMin = [-10; -4; -10; -4];
global_gMax = [10; 4; 10; 4];
global_gN = 65;
global_g = createGrid(global_gMin, global_gMax, global_gN);

positions = {[3 0], [-2 -2]};
angles = {0, pi/4};

[obsTotal, vMax] = plopObstacles(obsBRS_filenames, positions, angles, global_g);

figure
hold on

theta = linspace(0, 2*pi, 16);
vx = vMax*cos(theta);
vy = vMax*sin(theta);
for i = 1:length(vx)
  for j = 1:length(vy)
    vslice = [vx(i) vy(j)];
    [g2Dp, data2Dp] = proj(global_g, obsTotal, [0 1 0 1], vslice);
    visSetIm(g2Dp, data2Dp);
  end
end
end