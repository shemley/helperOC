tic
clear all
%% Calculate value function
% Grid parameters
Nx = 30; Ny = Nx; Ntheta = 30;
xmin = -10; xmax = 10;
ymin = -10; ymax = 10;
thetamin = 0; thetamax = 2*pi;

% calculate grid spacing
global x; x = linspace(xmin,xmax,Nx);
global y; y = linspace(ymin,ymax,Ny);
global theta; theta = linspace(thetamin,thetamax,Ntheta);

hx = diff(x); hx = hx(1);
hy = diff(y); hy = hy(1);
htheta = diff(theta); htheta = htheta(1);

% Turning radius
global rmin
rmin = 1;

% Value function
u = 1e9*ones(Nx,Ny,Ntheta);

% Target
%u(x>-1&x<1,y>-1&y<1,:) = 0;
u(x>-3&x<3,y>-3&y<3,:) = 0;

% number of iterations
N = 50;
for n = 1:N
    % in every node, do the following
    for i = 2:length(x)-1
        for j = 2:length(y)-1
            for k = 1:length(theta)
                % periodic boundary conditions for theta
                if k == 1
                    km1 = length(theta);
                else
                    km1 = k-1;
                end
                
                if k == length(theta)
                    kp1 = 1;
                else
                    kp1 = k+1;
                end
                               
                % equations 12 and 13
                xik = sign(cos(theta(k)));
                nuk = sign(sin(theta(k)));

                % (not really needed)
%                 cosux = -abs(cos(theta(k)))*(u(i+xik,j,k)-u(i,j,k))/hx;
%                 sinuy = -abs(sin(theta(k)))*(u(i,j+nuk,k)-u(i,j,k))/hy;

                % equation 14 (needed? maybe)
               % ur1 = (u(i,j,kp1)-u(i,j,k))/rmin/htheta;
               % ur2 = (u(i,j,km1)-u(i,j,k))/rmin/htheta;
               % ur = max([ur1 ur2 0]);

                % equation 15 and 16
                uss = abs(cos(theta(k)))*u(i+xik,j,k) + ...
                        abs(sin(theta(k)))*u(i,j+nuk,k) + hx;
                uss = uss/(abs(cos(theta(k)))+abs(sin(theta(k))));

%               hx could be hy here, doesn't matter if hx=hy=h
                us = abs(cos(theta(k)))*u(i+xik,j,k) + ...
                    abs(sin(theta(k)))*u(i,j+nuk,k) + ...
                    hx*min([u(i,j,kp1) u(i,j,km1)])/(rmin*htheta)+hx;
                us = us/(abs(cos(theta(k)))+abs(sin(theta(k)))+...
                    hx/(rmin*htheta));

                u(i,j,k) = min([u(i,j,k) us uss]);
                if(u(i,j,k) < 0 )
                    disp(['i,j,k =',num2str(i),', ' num2str(j),', ', num2str(k)]);
                    return;
                end


            end
        end
    end
    
end

%%
f1 = figure;
figure(f1)
clf;
%h1 = surf(x,y,u(:,:,4)); view(2)
[X,Y,Theta] = meshgrid(x,y,theta);
isosurface(X,Y,Theta,u,5);
axis([-10,10,-10,10,0,2*pi]);
%set(h1,'facealpha',0.3)
%%
toc

% %% Calculate trajectory
% % initial conditions
% x0 = -2; y0 = 1; theta0 = theta(4);
% IC = [x0 y0 theta0];
% 
% % time span (should take as much time as u says)
% ti = 0; tf = min([interpn(x,y,theta,u,x0,y0,theta0) 2]);
% tspan = [ti tf];
% 
% % compute partial derivative u_theta for equation 19
% global utheta
% utheta = zeros(size(u));
% for i = 1:length(x)
%     for j = 1:length(y)
%         for k = 1:length(theta)
%                 % periodic boundary conditions for theta
%                 if k == 1
%                     km1 = length(theta);
%                 else
%                     km1 = k-1;
%                 end
%                 
%                 if k == length(theta)
%                     kp1 = 1;
%                 else
%                     kp1 = k+1;
%                 end
%                 
%                 % calculate derivative
%                 utheta(i,j,k) = (u(i,j,kp1)-u(i,j,km1))/2/htheta;
%         end
%     end
% end
% 
% % Use ODE solver to generate trajectory
% [T F] = ode45(@Dubinfun, tspan, IC); % angle wrap problem...
% figure(f1)
% hold on
% h2 = plot(F(:,1),F(:,2),'b.');