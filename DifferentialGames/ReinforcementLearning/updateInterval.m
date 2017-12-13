function intStruct = updateInterval(intStruct,newVal,alpha)

% Calculate new values
newN    = intStruct.n + 1; 
newMean = (intStruct.mean*intStruct.n + newVal)/newN;
newVar  = ((intStruct.n-2)*intStruct.var + ...
           (newVal - newMean)*(newVal - intStruct.mean))/(intStruct.n-1);
       
% Store values
intStruct.n = newN;
intStruct.mean = newMean;
intStruct.var = newVar;

% Update intervals
t_crit = tinv(1-alpha/2,intStruct.n-1);
width = t_crit*sqrt(intStruct.var/intStruct.n);
intStruct.lower = intStruct.mean - width;
intStruct.upper = intStruct.mean + width;
