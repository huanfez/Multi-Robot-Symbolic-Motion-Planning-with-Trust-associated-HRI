clear,clc
% Original Environment
gridWidth = 10;
gridLength = 10;
start = 1;
obstacles = [13, 56, 86, 88, 77, 24, 35, 45];
goals = [25, 87, 54];

%% Envr Translation
[xObsInt,yObsInt] = cellPath2Grid(obstacles,gridWidth,gridLength);
[xGoalInt,yGoalInt] = cellPath2Grid(goals,gridWidth,gridLength);
[xStartInt,yStartInt] = cellPath2Grid(start,gridWidth,gridLength);

gridWidth = 100;
gridLength = 100;
% gridChunk = [1 gridWidth 1 gridLength];
gridChunk = [1 50 1 50];
xTrans = 0;
yTrans = 0;

obstacles = gridPath2Cell(xObsInt+xTrans,yObsInt+yTrans,gridWidth,gridLength);
goals = gridPath2Cell(xGoalInt+xTrans,yGoalInt+yTrans,gridWidth,gridLength);
start = gridPath2Cell(xStartInt+xTrans,yStartInt+yTrans,gridWidth,gridLength);

%% Resume Environment
goalsReached = [];
goalsLeft = goals;
ltlspec = ['LTLSPEC ! (( F (x.state = ',num2str(goals(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
for i = 2:length(goals)
    ltlspec = [ltlspec, ' & F (x.state = ',num2str(goals(i)),')'];
end
ltlspec = [ltlspec, ' ))'];

% Model Checker
fileName = 'MotionTest.smv';
% sensedObstacles = []; % before implementing, device does not see obstacles
sensedObstacles = obstacles;
[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goals, gridWidth, gridLength);
[xStart,yStart] = cellPath2Grid(start, gridWidth, gridLength);

%% Plot Enviornmnet

% xText = [];
% yText = [];
% for i = 1:gridLength
%     xText = [xText, 1:gridWidth];
%     yText = [yText, i*ones(1, gridWidth)];
% end
% 
% xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
% yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;
% 
% % Full grid path
% figure(3)
% hold off
% plot(xGrid,yGrid,'k')
% axis equal
% axis([0 gridWidth+1 0 gridLength+1])
% hold on
% % text(xText+.1,yText+.25,num2str([1:gridWidth*gridLength]'))
% plot(xGoal, yGoal, 'dm', 'MarkerSize', 12)
% plot(xObs,yObs,'xr','MarkerSize',15)
% plot(xStart,yStart,'ok','MarkerSize',12)
% title('Pathing'),xlabel('x'),ylabel('y')
% hold off
% 
% pause(.01)

%% Planning
filePath = cd;
pathNuSMV = 'C:\Program Files\NuSMV\2.5.4\bin';
tic
makeSMV_v2(fileName, gridWidth, gridLength, gridChunk, start, sensedObstacles, ltlspec);
toc, tic
[~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
toc
[xPath,yPath,cPath] = getPath(output, gridWidth, gridLength);

%% Plot inital plan
% figure(1)
% plot(xGrid,yGrid,'k')
% hold on
% plot(xPath(1),yPath(1),'ko',xPath(end),yPath(end),'k^','MarkerSize',12)
% axis equal
% axis([0 gridWidth+1 0 gridLength+1])
% arrow([xPath(1:end-1)',yPath(1:end-1)'],...
%     [xPath(2:end)',yPath(2:end)'],10);
% % text(xPath+.1,yPath+.1,num2str(cPath'))
% plot(xGoal, yGoal, 'dm', 'MarkerSize', 12)
% plot(xSObs,ySObs,'xr','MarkerSize',15)
% axis equal
% axis([0 gridWidth+1 0 gridLength+1])
% title('Planning'),xlabel('x'),ylabel('y')
% hold off
% 













