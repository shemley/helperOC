function [newAnodes newNnodes newFnodes newV newNodes newVtable newfbCtrl newtCtrl] = RSCarUpdate(Anodes, Nnodes, Fnodes, x, y, theta, V, oldNodes, oldVtable, c, oldfbCtrl, oldtCtrl)
% Update function. Adds one accepted node to the grid
% --- inputs ---
% Anodes        matrix of accepted nodes
% Nnodes        matrix of narrow-band nodes
% Fnodes        matrix of faraway nodes
% V             value function
% c             flow speed function
% oldNodes      values have to be updated for these nodes this iteration
% oldValues     table of previous narrow-band values
%
% --- outputs ---
% newAnodes     matrix of accepted nodes
% newNnodes     matrix of narrow-band nodes
% newFnodes     matrix of faraway nodes
% newV          value function
% newNodes      values have to be updated for these nodes next iteration
% newVtable     table of new narrow-band values

% Neighbour information
global neighbours
global endPts

dx = x(2)-x(1);
dy = y(2)-y(1);
dtheta = theta(2)-theta(1);

[X Y Theta] = meshgrid(x,y,theta);

% Copy control storage variable
newfbCtrl = oldfbCtrl;
newtCtrl = oldtCtrl;

% Calculate new potential value for each neighbour
% [Ni Nj Nk] = ind2sub(size(Nnodes),find(Nnodes));
if size(oldNodes,1) % if there are new narrow-band nodes to compute
    Ni = oldNodes(:,1); Nj = oldNodes(:,2); Nk = oldNodes(:,3);

    % copy value function
    Vcopy = V;

    for l = 1:length(Ni)
        i = Ni(l); j = Nj(l); k = Nk(l);
        


        % update value function
        for n = 1:2 % forward and backward
        switch n
            case 1
                forward = -1;
            case 2
                forward = 1;
        end

            for m = 1:3    % turn left, turn right, go straight
                switch m
                    case 2
                        turn = 0;
                        % update value depending on whether we're going forward
                        % or backwards
                        if i > 1 && j > 1 && i < length(x) && j < length(y)
                            if theta(k) >= -pi && theta(k) < -3*pi/4
                                dt = abs(dx/c(i,j,k)./cos(theta(k)));

                                a = abs(tan(theta(k)));
                                
                                newValue = dt+(1-a)*V(i-forward,j,k) + a*V(i-forward,j-1,k);
                                
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end
                                
                            elseif theta(k) >= -3*pi/4 && theta(k) < -2*pi/4
                                dt = abs(dx/c(i,j,k)./sin(theta(k)));
                                
                                a = abs(cot(theta(k)));
                                
                                newValue = dt+(1-a)*V(i,j-forward,k) + a*V(i-1,j-forward,k);
                                
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end

                            elseif theta(k) >= -2*pi/4 && theta(k) < -pi/4
                                dt = abs(dx/c(i,j,k)./sin(theta(k)));

                                a = abs(cot(theta(k)));
                                
                                newValue = dt+(1-a)*V(i,j-forward,k) + a*V(i+1,j-forward,k);
                                
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end

                            elseif theta(k) >= -pi/4 && theta(k) < 0
                                dt = abs(dx/c(i,j,k)./cos(theta(k)));
 
                                a = abs(tan(theta(k)));
                                
                                newValue = dt+(1-a)*V(i+forward,j,k) + a*V(i+forward,j-1,k);
                                
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end


                            elseif theta(k) >= 0 && theta(k) < pi/4
                                dt = abs(dx/c(i,j,k)./cos(theta(k)));
                                
                                a = abs(tan(theta(k)));
                                
                                newValue = dt+(1-a)*V(i+forward,j,k) + a*V(i+forward,j+1,k);

                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end

                            elseif theta(k) >= pi/4 && theta(k) < 2*pi/4
                                dt = abs(dx/c(i,j,k)./sin(theta(k)));

                                a = abs(cot(theta(k)));
                                
                                newValue = dt+(1-a)*V(i,j+forward,k) + a*V(i+1,j+forward,k);

                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end

                            elseif theta(k) >= 2*pi/4 && theta(k) < 3*pi/4
                                dt = abs(dx/c(i,j,k)./sin(theta(k)));

                                a = abs(cot(theta(k)));
                                
                                newValue = dt+(1-a)*V(i,j+forward,k) + a*V(i-1,j+forward,k);
                                                         
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end
                                
                            else
                                dt = abs(dx/c(i,j,k)./cos(theta(k)));

                                a = abs(tan(theta(k)));
                                
                                newValue = dt+(1-a)*V(i-forward,j,k) + a*V(i-forward,j+1,k);
                                
                                if newValue < Vcopy(i,j,k)
                                    Vcopy(i,j,k) = newValue;
                                    newfbCtrl(i,j,k) = forward;
                                    newtCtrl(i,j,k) = turn;
                                end
                            end
                        end
                            if Vcopy(i,j,k) < 0
                                error('Value is negative! straight')
                            end                    
                    otherwise
                        switch m
                            case 1
                                turn = -1;
                            case 3
                                turn = 1;
                        end
                        
                        dt = dtheta;

                        leftBot = [i j k] + neighbours{m,n,k}(1,:);
                        leftTop = [i j k] + neighbours{m,n,k}(2,:);
                        rightBot = [i j k] + neighbours{m,n,k}(3,:);
                        rightTop = [i j k] + neighbours{m,n,k}(4,:);

                        % periodic boundary conditions for theta
                        if leftBot(3) > length(theta)
                            leftBot(3) = 1;
                            leftTop(3) = 1;
                            rightBot(3) = 1;
                            rightTop(3) = 1;
                        elseif leftBot(3) < 1
                            leftBot(3) = length(theta);
                            leftTop(3) = length(theta);
                            rightBot(3) = length(theta);
                            rightTop(3) = length(theta);
                        end

                        middle = endPts{m,n,k};

                        a = middle(1)-neighbours{m,n,k}(1,1);
                        b = middle(2)-neighbours{m,n,k}(1,2);

                        % Update unless we're at the edge of the grid
                        if leftBot(1) > 1 && leftBot(2) > 1 && rightTop(1) < length(x) && rightTop(2) < length(y) 

                            newValue = ((1-a)*(1-b)*V(leftBot(1),leftBot(2),leftBot(3)) + ...
                                (1-a)*b*V(leftTop(1),leftTop(2),leftTop(3)) + ...
                                a*(1-b)*V(rightBot(1),rightBot(2),rightBot(3)) + ...
                                a*b*V(rightTop(1),rightTop(2),rightTop(3))) + dt;
                            
                            if newValue < Vcopy(i,j,k)
                                Vcopy(i,j,k) = newValue;
                                newfbCtrl(i,j,k) = forward;
                                newtCtrl(i,j,k) = turn;
                            end

                            if Vcopy(i,j,k) < 0
                                error('Value is negative! turn')
                            end
                        end
                end 
            end
        end
    end
end

% check which potential new function value is minimum, and update that
newVtable = oldVtable;
if size(oldNodes,1) % if there are new narrow-band nodes to compute
    for l = 1:length(Ni)
%         if Vcopy(Ni(l),Nj(l),Nk(l)) < 1e6
            newVtable = [newVtable; Ni(l) Nj(l) Nk(l) Vcopy(Ni(l),Nj(l),Nk(l))];
%         end
    end
end
newVtable = sortrows(-newVtable,4); % Sort -newVtable in asending order
newVtable = -newVtable;             % now newVtable is sorted in descending order

% newVtable(end-3:end, 1:3)
% minNewValInd = newVtable(find(newVtable(:,4)==min(newVtable(:,4)),1,'first'),1:3);
% V(minNewValInd(1), minNewValInd(2), minNewValInd(3)) = min(newVtable(:,4));

% now the minimum value is in the last row of Vtable
minNewValInd = [newVtable(end,1) newVtable(end,2) newVtable(end,3)];
newestValue = newVtable(end,4);
V(minNewValInd(1), minNewValInd(2), minNewValInd(3)) = newestValue;

% delete rows with duplicate indices
deleteInd = find(newVtable(:,1) == minNewValInd(1) & ...
    newVtable(:,2) == minNewValInd(2) & ...
    newVtable(:,3) == minNewValInd(3));
newVtable(deleteInd,:) = [];




%% Update accepted nodes
Anodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)) = 1;

%% Update narrow-band and faraway nodes
Nnodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)) = 0; % current node is no longer narrow-band

% Check neighbouring nodes to see if they are now new narrow-band nodes and
% update Vtable and newNodes
% [minNewValInd(1) minNewValInd(2) minNewValInd(3) newestValue]
newNodes = [];
if minNewValInd(1) < length(x)-1
    if ~Anodes(minNewValInd(1)+1, minNewValInd(2), minNewValInd(3))
        Nnodes(minNewValInd(1)+1, minNewValInd(2), minNewValInd(3)) = 1;
        Fnodes(minNewValInd(1)+1, minNewValInd(2), minNewValInd(3)) = 0;
        newNodes = [newNodes; minNewValInd(1)+1, minNewValInd(2), minNewValInd(3)];
    end
end

if minNewValInd(1) > 2
    if ~Anodes(minNewValInd(1)-1, minNewValInd(2), minNewValInd(3))
        Nnodes(minNewValInd(1)-1, minNewValInd(2), minNewValInd(3)) = 1;
        Fnodes(minNewValInd(1)-1, minNewValInd(2), minNewValInd(3)) = 0;
        newNodes = [newNodes; minNewValInd(1)-1, minNewValInd(2), minNewValInd(3)];
    end
end

if minNewValInd(2) < length(y)-1
    if ~Anodes(minNewValInd(1), minNewValInd(2)+1, minNewValInd(3))
        Nnodes(minNewValInd(1), minNewValInd(2)+1, minNewValInd(3)) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2)+1, minNewValInd(3)) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2)+1, minNewValInd(3)];
    end
end

if minNewValInd(2) > 2
    if ~Anodes(minNewValInd(1), minNewValInd(2)-1, minNewValInd(3))
        Nnodes(minNewValInd(1), minNewValInd(2)-1, minNewValInd(3)) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2)-1, minNewValInd(3)) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2)-1, minNewValInd(3)];
    end
end

if minNewValInd(3) < length(theta)
    if ~Anodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)+1)
        Nnodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)+1) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)+1) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2), minNewValInd(3)+1];
    end
end

if minNewValInd(3) > 1
    if ~Anodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)-1)
        Nnodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)-1) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2), minNewValInd(3)-1) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2), minNewValInd(3)-1];
    end
end

if minNewValInd(3) == length(theta)
    if ~Anodes(minNewValInd(1), minNewValInd(2), 1)
        Nnodes(minNewValInd(1), minNewValInd(2), 1) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2), 1) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2), 1];
    end
end

if minNewValInd(3) == 1
    if ~Anodes(minNewValInd(1), minNewValInd(2), length(theta))
        Nnodes(minNewValInd(1), minNewValInd(2), length(theta)) = 1;
        Fnodes(minNewValInd(1), minNewValInd(2), length(theta)) = 0;
        newNodes = [newNodes; minNewValInd(1), minNewValInd(2), length(theta)];
    end
end

newAnodes = Anodes;
newNnodes = Nnodes;
newFnodes = Fnodes;
newV = V;

