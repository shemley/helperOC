function [Anodes Nnodes Fnodes u] = RSCarInit(x, y, theta, targetx, targety, targettheta)
% Initialization function. Creates matrices representing accepted,
% narrow-band, and faraway nodes, as well as the value function.
%
% --- inputs ---
% x:            vector of x values
% y:            vector of y values
% theta:        vector of theta values
% targetx:      target set in the form [xmin xmax]
% targety:      target set in the form [ymin ymax]
% targettheta:  target set in the form [thetamin thetamax]
%
% --- outputs ---
% Anodes        matrix of accepted nodes
% Nnodes        matrix of narrow-band nodes
% Fnodes        matrix of faraway nodes
% u             value function

%% initialize accepted nodes
[X Y Theta] = meshgrid(x,y,theta);
Anodes = zeros(size(X));
% Anodes(x>=targetx(1)&x<=targetx(2), y>=targety(1)&y<=targety(2), ...
%     theta>=targettheta(1)&theta<=targettheta(2))=1;
% Anodes(X.^2+Y.^2<0.2)=1;

% Boundary target
Anodes(:,2,:) = 1;
Anodes(:,end-1,:) = 1;
Anodes(2,:,:) = 1;
Anodes(end-1,:,:) = 1;
%% initialize narrow-band nodes
Nnodes = zeros(size(X));
[Ai Aj Ak] = ind2sub(size(Anodes),find(Anodes));

for l = 1:length(Ai)
    i = Ai(l); j = Aj(l); k = Ak(l);

    % Neighbours in the x direction
    if i<length(x)
        if ~Anodes(i+1,j,k)
            Nnodes(i+1,j,k)=1;
        end
    end

    if i>1
        if ~Anodes(i-1,j,k)
            Nnodes(i-1,j,k)=1;
        end
    end

    % Neighbours in the y direction
    if j<length(y)
        if ~Anodes(i,j+1,k)
            Nnodes(i,j+1,k)=1;
        end
    end

    if j>1
        if ~Anodes(i,j-1,k)
            Nnodes(i,j-1,k)=1;
        end
    end
    
    % Neighbours in the theta direction, with boundary conditions
    if k==length(theta)
        kp1 = 1;
    else
        kp1 = k+1;
    end
    
    if k==1
        km1 = length(theta);
    else
        km1 = k-1;
    end
    
    if ~Anodes(i,j,kp1)
        Nnodes(i,j,kp1)=1;
    end
    
    if ~Anodes(i,j,km1)
        Nnodes(i,j,km1)=1;
    end
end

%% Initialize faraway nodes
Fnodes = ~Anodes & ~Nnodes;

%% Initialize value function
u = 1e9*ones(length(x),length(y),length(theta));
u(Anodes==1)=0;