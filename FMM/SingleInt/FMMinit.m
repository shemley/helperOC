function [Anodes Nnodes Fnodes V] = FMMinit(x, y, targetx, targety)
% Initialization function. Creates matrices representing accepted,
% narrow-band, and faraway nodes, as well as the value function.
%
% --- inputs ---
% x:            vector of x values
% y:            vector of y values
% targetx:      target set in the form [xmin xmax]
% targety:      target set in the form [ymin ymax]
% --- outputs ---
% Anodes        matrix of accepted nodes
% Nnodes        matrix of narrow-band nodes
% Fnodes        matrix of faraway nodes
% V             value function

%% initialize accepted nodes
Anodes = zeros(length(x),length(y));
Anodes(x>targetx(1)&x<targetx(2), y>targety(1)&y<targety(2))=1;

%% initialize narrow-band nodes (use a function maybe)
Nnodes = zeros(length(x),length(y));
[Ar Ac] = find(Anodes==1);      % locate accepted nodes

for l = 1:length(Ar)
    i = Ar(l); j = Ac(l);

    if i<length(x)
        if ~Anodes(i+1,j)
            Nnodes(i+1,j)=1;
        end
    end

    if i>1
        if ~Anodes(i-1,j)
            Nnodes(i-1,j)=1;
        end
    end

    if j<length(y)
        if ~Anodes(i,j+1)
            Nnodes(i,j+1)=1;
        end
    end

    if j>1
        if ~Anodes(i,j-1)
            Nnodes(i,j-1)=1;
        end
    end
end

%% Initialize faraway nodes
Fnodes = ~Anodes & ~Nnodes;

%% Initialize value function
V = 1e9*ones(length(x),length(y));
V(Anodes==1)=0;