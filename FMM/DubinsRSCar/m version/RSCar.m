%% Initialize grid
% State space
% Nx = 15; Ny = Nx; Ntheta = Nx;
% x = linspace(-5,5,Nx);
% y = linspace(-5,5,Ny);
% theta = linspace(-pi,pi,Ntheta);
% 
% [X Y Theta] = meshgrid(x,y,theta);  % for convenience
tic
%%
% Target set
targetx = [-0.2 0.2]; targety = [-0.2 0.2]; targettheta = [-10 10];

% Nodes
[Anodes Nnodes Fnodes u] = RSCarInit(x, y, theta, targetx, targety, targettheta);
[Ni Nj Nk] = ind2sub(size(Nnodes),find(Nnodes));
Nodes = [Ni Nj Nk];
Vtable = []; % dummy value for Vtable

% Variables to store control
fbCtrl = -2*ones(size(X));
tCtrl = -2*ones(size(X));

% Execute update loop at least once
[Anodes Nnodes Fnodes u Nodes Vtable] = RSCarUpdate(Anodes, Nnodes, Fnodes, x, y, theta, u, Nodes, Vtable, c, fbCtrl, tCtrl);
    
%% Update value function
% while sum(sum(sum(Anodes))) < (length(x)-2)*(length(y)-2)*length(theta)

dummy = 0;
while min(Vtable(:,4)) < 1e6
%     dummy = dummy+1;
%     if dummy >= 500
%         return;
%     end
    [Anodes Nnodes Fnodes u Nodes Vtable fbCtrl tCtrl] = RSCarUpdate(Anodes, Nnodes, Fnodes, x, y, theta, u, Nodes, Vtable, c, fbCtrl, tCtrl);
end

%% Plot
% figure;
% k = 8; j = 8;
% FinalValue = u(2:end-1,j,k);
% plot(x(2:end-1),FinalValue.*(FinalValue<10))
% xlabel('x')
% title(['y = ' num2str(y(k)) ', \theta = ' num2str(theta(k))])
% % axis square
% 
% figure;
% k = 8; i = 8;
% FinalValue = u(i,2:end-1,k);
% plot(y(2:end-1),FinalValue.*(FinalValue<10))
% xlabel('y')
% title(['x = ' num2str(x(k)) ', \theta = ' num2str(theta(k))])
% % axis square
% 
% figure;
% k = 8;
% FinalValue = u(2:end-1,2:end-1,k);
% contour(x(2:end-1),y(2:end-1),FinalValue.*(FinalValue<10))
% title(['\theta = ' num2str(theta(k))])
% xlabel('x')
% ylabel('y')
% view(2)
% % axis square

%%
figure;
FinalValue = u(2:end-1,2:end-1,:);
isosurface(X(2:end-1,2:end-1,:),Y(2:end-1,2:end-1,:),Theta(2:end-1,2:end-1,:), FinalValue,0.3)
xlabel('x')
ylabel('y')
xlim([min(x) max(x)])
ylim([min(y) max(y)])
zlim([min(theta) max(theta)])
title(['Nx = ' num2str(Nx)])
% axis square
%%
figure;
FinalValue2 = min(FinalValue,[],3);
contour(x(2:end-1),y(2:end-1),FinalValue2)

%%
toc