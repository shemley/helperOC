function RS_illustration()
% unitCircle()
%
% Plots an implicit surface function for the unit disk

%% Grid and function
N = 201;
L = 2.5;
X = linspace(-L, L, N);
Y = linspace(-L, L, N);
[x,y] = ndgrid(X,Y);

l = sqrt(x.^2 + y.^2) - 1;

%% Plot surface of function
f = figure;
f.Color = 'white';
ll = surf(x,y,l);
ll.FaceAlpha = 0.5;
ll.LineStyle = 'none';
ll.FaceColor = 'g';
hold on

%% Plot unit circle
theta = linspace(0, 2*pi, 100);
xx = cos(theta);
yy = sin(theta);
zz = zeros(size(xx));
p = plot3(xx,yy,zz);
p.Color = 'r';
p.LineWidth = 3;

%% Plot surface of ellipse
V = sqrt((x + 0.5).^2 / 1.75^2 + y.^2 / 1.25^2) - 1;
W = surf(x, y, V);
W.FaceAlpha = 0.5;
W.LineStyle = 'none';
W.FaceColor = 'b';

%% Plot ellipse
xx = -0.5 + 1.75*cos(theta);
yy = 1.25 * sin(theta);
e = plot3(xx,yy,zz);
e.Color = 'b';
e.LineWidth = 2;

% axis labels
xlabel('x')
ylabel('y')
xlim([-L L])
ylim([-L L])
axis equal
lighting phong
camlight left
camlight right
% savefig(f, sprintf('%s.fig', mfilename), 'compact')
% export_fig(sprintf('%s', mfilename), '-pdf', '-transparent')
end