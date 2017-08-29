function dirn = shortestPathDirection(g, u, x)
% dirn = shortestPathDirection(g, u, x)
% Computes optimal direction along the shortest path
%
% NaN values are set to infinity
%

% Gradient in x direction
uxm = eval_u(u,x(1)-g.dx(1),x(2),g);
uxp = eval_u(u,x(1)+g.dx(1),x(2),g);
ux = eval_u(u,x(1),x(2),g);

duxm = ux - uxm;
duxp = uxp - ux;

if duxm * duxp > 0
    if uxp > uxm,   dux = duxm;
    else            dux = duxp; end
else
    if duxm > 0
        if uxp > uxm,   dux = duxm;
        else            dux = duxp; end
    else                dux = 0;
    end
end


% Gradient in y direction
uym = eval_u(u,x(1),x(2)-g.dx(2),g);
uyp = eval_u(u,x(1),x(2)+g.dx(2),g);
uy = eval_u(u,x(1),x(2),g);

duym = uy - uym;
duyp = uyp - uy;

if duym * duyp > 0
    if uyp > uym,   duy = duym;
    else            duy = duyp; end
else
    if duym > 0
        if uyp > uym,   duy = duym;
        else            duy = duyp; end
    else                duy = 0;
    end
end


dirn = -[dux duy];

if norm(dirn) > 0
    dirn = dirn/norm(dirn);
end

end