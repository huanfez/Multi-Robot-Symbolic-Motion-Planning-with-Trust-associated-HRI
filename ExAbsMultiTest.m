clear, clc
%% Environment
gridWidth = 10;
gridLength = 10;
start = [12;79]; % agent per row
obstacles = [13, 56, 86, 88, 77, 24, 35, 45];
goalsAll = [25, 87, 54, 84];
goals = [{[25, 54]}; {[84, 87]}];% agent per row

N = 2; % Number of agents
goalsReached = cell(N,1);
goalsLeft = goals;

%% Model Checker Parameters
ltlspec = cell(N,1);
for n = 1:N
    ltlspec{n} = ['LTLSPEC ! (( F (x.state = ',num2str(goals{n}(1)),')'];
    for i = 2:length(goals{n})
        ltlspec{n} = [ltlspec{n}, ' & F (x.state = ',num2str(goals{n}(i)),')'];
    end
    ltlspec{n} = [ltlspec{n}, ' ))'];
end

% file locations
fileName = 'MultiTest.smv';
filePath = cd;
pathNuSMV = 'C:\Program Files\NuSMV\2.5.4\bin';

sensedObstacles = obstacles;
[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goalsAll, gridWidth, gridLength);
[xStart,yStart] = cellPath2Grid(start', gridWidth, gridLength);

%% Plot Enviornmnet
for dummy = 1:1
% xText = [];
% yText = [];
% for i = 1:gridLength
%     xText = [xText, 1:gridWidth];
%     yText = [yText, i*ones(1, gridWidth)];
% end

xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;

figure(1)
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
hold on
% text(xText+.1,yText+.25,num2str([1:gridWidth*gridLength]'))
plot(xGoal, yGoal, 'dm', 'MarkerSize', 12)
plot(xObs,yObs,'xr','MarkerSize',15)
plot(xStart,yStart,'ok','MarkerSize',12)
title('Pathing'),xlabel('x'),ylabel('y')
hold off

pause(.01)
end

%% Planning
xPath = cell(N,1);
yPath = cell(N,1);
cPath = cell(N,1);
gridChunk = zeros(N,4);

for n = 1:N
    expansion = -1;
    while isempty(cPath{n})
        
        expansion = expansion + 1;
        
        gridChunk(n,:) = getChunk(start(n), goals{n}, gridWidth, gridLength,expansion);
        
        hold on
        rectangle('Position',[gridChunk(n,1)-.5, gridChunk(n,3)-.5, ...
            gridChunk(n,2)-gridChunk(n,1)+1, gridChunk(n,4)-gridChunk(n,3)+1],...
            'LineWidth',6)
        hold off
        
        pause(.0001)
        
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(n,:), start(n), sensedObstacles, ltlspec{n});
        
        [~, output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
        
        [xPath{n},yPath{n},cPath{n}] = getPath(output, gridWidth, gridLength);
        
    end
    
end

%% Plot initial plan

figure(2)
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Planning'),xlabel('x'),ylabel('y')
hold on
plot(xGoal,yGoal,'dm','MarkerSize',12)
plot(xSObs,ySObs,'xr','MarkerSize',15)
for n = 1:N
    plot(xPath{n}(1),yPath{n}(1),'ko',xPath{n}(end),yPath{n}(end),'k^',...
        'MarkerSize',12)
    arrow([xPath{n}(1:end-1)',yPath{n}(1:end-1)'],...
    [xPath{n}(2:end)',yPath{n}(2:end)'],10);
end
hold off

















