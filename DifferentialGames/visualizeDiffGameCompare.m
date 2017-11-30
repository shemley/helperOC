function visualizeDiffGameCompare(OL,CL,MPC,gCL,gOL,extraArgs)
% Function to visualize the output of the differential game comparison
% function. Compares open loop, closed

extraArgsCL = extraArgs;
extraArgsOL = extraArgs;
extraArgsMPC = extraArgs;

% Dimensions to show in the plot
showDims = [1 1 0 0]; 

% CL extra args
extraArgsCL.projDim = showDims;
extraArgsCL.distDim = ~showDims;
extraArgsCL.targetData = CL.targetData;
extraArgsCL.obstacleData = CL.obstacles;

% OL extra args
extraArgsOL.trajDim = showDims;
extraArgsOL.projDim = showDims;
extraArgsOL.distDim = ~showDims;
extraArgsOL.targetData = OL.targetData;
% obstacles in loop

% MPC extra args
extraArgsMPC.trajDim = showDims;
extraArgsMPC.projDim = showDims;
extraArgsMPC.distDim = ~showDims;
extraArgsMPC.targetData = MPC.targetData;
% obstacles in loop


% Flip data and obstacles
% CL
dataTrajCL = flip(CL.data,ndims(CL.data));

% % Closed loop
% CL.AX = AX;
% CL.AY = AY;
% CL.DX = DX;
% CL.DY = DY;
% CL.Values = nan(size(AX));
% CL.trajectories = cell(size(AX));
% CL.data = dataCL;
% CL.obstacles = obstaclesCL;
% CL.targetData = targetDataCL;
% 
% % Open loop
% OL.AX = AX;
% OL.AY = AY;
% OL.DX = DX;
% OL.DY = DY;
% OL.Values = nan(size(AX));
% OL.trajectories = cell(size(AX));
% OL.data = cell(size(AX));
% OL.obstacles = cell(size(AX));
% OL.targetData = targetDataOL;
% 
% % MPC
% MPC.horizons = horizons;
% MPC.AX = AX;
% MPC.AY = AY;
% MPC.DX = DX;
% MPC.DY = DY;
% MPC.Values = nan([size(AX),length(horizons)]);
% MPC.trajectories = cell([size(AX),length(horizons)]);
% MPC.data = cell([size(AX),length(horizons)]);
% MPC.obstacles = cell([size(AX),length(horizons)]);
% MPC.targetData = targetDataOL;




% Animate results in 3 plot figure
f = figure('units','normalized','outerposition',[0 0 1 1]); 

for iax = 1%:size(CL.AX,1)
  for iay = 1%:size(CL.AY,2)
    for idx = 1%:size(CL.DX,3)
      for idy = 1%:size(CL.DY,4)
        subplot(1,3,1); title('CL');
        subplot(1,3,2); title('OL');
        subplot(1,3,3); title('MPC');
        
        % Flip OL data
        dataTrajOL = flip(OL.data{iax,iay,idx,idy},...
                               ndims(OL.data{iax,iay,idx,idy}));
        
        % animate CL
        subplot(1,3,1)
        axis square
        axis equal        
        visualizeOptTraj(gCL,dataTrajCL,CL.trajectories{iax,iay,idx,idy}.x,...
                         CL.trajectories{iax,iay,idx,idy}.tau, extraArgsCL)
        title('CL')
        
        % animate OL
        extraArgsOL.obstacleData = flip(OL.obstacles{iax,iay,idx,idy},...
                                   ndims(OL.obstacles{iax,iay,idx,idy}));
        
        subplot(1,3,2)
        axis square
        axis equal        
        visualizeOptTraj(gOL,dataTrajOL,...
                         OL.trajectories{iax,iay,idx,idy}.x,...
                         OL.trajectories{iax,iay,idx,idy}.tau, extraArgsOL)
        title('OL')
        
        % animate MPC for each horizon
        
        subplot(1,3,3)
        axis square
        axis equal        
        for ihz = length(MPC.horizons):-1:1         
            % Flip MPC data
            dataTrajMPC = flip(MPC.data{iax,iay,idx,idy,ihz},...
                            ndims(MPC.data{iax,iay,idx,idy,ihz}));
            extraArgsMPC.obstacleData = flip(MPC.obstacles{iax,iay,idx,idy,ihz},...
                            ndims(MPC.obstacles{iax,iay,idx,idy,ihz}));
            visualizeOptTraj(gOL,dataTrajMPC,...
                MPC.trajectories{iax,iay,idx,idy,ihz}.x,...
                MPC.trajectories{iax,iay,idx,idy,ihz}.tau, extraArgsMPC)
        end    
        title(['MPC with horizon of ',num2str(MPC.horizons(ihz))]) 
      end
    end
  end
end
