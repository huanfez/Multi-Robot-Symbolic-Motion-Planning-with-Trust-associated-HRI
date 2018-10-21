clear, clc
timeStart = tic;
MakeMovie = 0;
MovieName = 'ExAbsMultiTest_RT.avi';
%% Environment

gridWidth = 10;
gridLength = 10;
start = [11;91;20]; % agent per row
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 94, 85, 80, 70, 69, 48, 38];
goals = [{[25, 54]}; {[84, 87]}; {[17, 100]}];% agent per row

M = 3; % Number of agents

goalsAll = [];
for m = 1:M
    goalsAll = [goalsAll goals{m}];
end

goalsReached = cell(M,1);
goalsReachedAll = [];
goalsLeft = goals;

%% Agents
% Time
dt = .02;
T = 1000; % sec

N = T/dt; % discrete horizon

% Dynamics
A = eye(2);
B = dt*eye(2);
Q = eye(2);
R = eye(2);
x0 = cell(M,1);

for m = 1:M
    [xInt,yInt] = cellPath2Grid(start(m),gridWidth,gridLength);
    x0{m} = [xInt;yInt];
end

goalTol = .125;

x = cell(M,1);
u = cell(M,1);
for m = 1:M
    x{m} = zeros(2,N); x{m}(:,1) = x0{m};
    u{m} = zeros(2,N-1);
end

K = dlqr(A,B,Q,R); % u = -Kx, x(i+1) = Ax(i) + Bu(i)

%% Model Checker Parameters

ltlspec = cell(M,1);
for m = 1:M
    ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goals{m}(1)),')'];
    for i = 2:length(goals{m})
        ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goals{m}(i)),')'];
    end
    ltlspec{m} = [ltlspec{m}, ' ))'];
end

% file locations
fileName = 'MultiTest.smv';
filePath = cd;
pathNuSMV = 'C:\Program Files\NuSMV\2.5.4\bin';

% sensedObstacles = obstacles;
sensedObstacles = [];
[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goalsAll, gridWidth, gridLength);
[xStart,yStart] = cellPath2Grid(start', gridWidth, gridLength);

%% Plot Enviornmnet

% xText = [];
% yText = [];
% for i = 1:gridLength
%     xText = [xText, 1:gridWidth];
%     yText = [yText, i*ones(1, gridWidth)];
% end

xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;

figure(1)
subplot(1,2,1)
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
hold on
% text(xText+.1,yText+.25,num2str([1:gridWidth*gridLength]'))
plot(xGoal, yGoal, 'dm', 'MarkerSize', 12)
plot(xSObs,ySObs,'xr','MarkerSize',15)
plot(xStart,yStart,'ok','MarkerSize',12)
title('Pathing'),xlabel('x'),ylabel('y')
hold off

pause(.1)


%% Planning

xPath = cell(M,1);
yPath = cell(M,1);
cPath = cell(M,1);
gridChunk = zeros(M,4);
mTrack = cell(M,1);

for m = 1:M
    expansion = -1;
    while isempty(cPath{m})
        
        expansion = expansion + 1;
        
        gridChunk(m,:) = getChunk(start(m), goals{m}, gridWidth, gridLength,expansion);
        
        hold on
        rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
            gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
            'LineWidth',6)
        hold off
        
        pause(.1)
        
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), start(m), sensedObstacles, ltlspec{m});
        
        [~, output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
        
        [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
        
    end
    
    % For collision detection, save current and next cells
    mTrack{m} = cPath{m}(1:2);
    
end

%% Plot initial plan

% figure(2)
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Planning'),xlabel('x'),ylabel('y')
hold on
plot(xGoal,yGoal,'dm','MarkerSize',12)
plot(xSObs,ySObs,'xr','MarkerSize',15)
for m = 1:M
    plot(xPath{m}(1),yPath{m}(1),'ko',xPath{m}(end),yPath{m}(end),'k^',...
        'MarkerSize',12)
    arrow([xPath{m}(1:end-1)',yPath{m}(1:end-1)'],...
        [xPath{m}(2:end)',yPath{m}(2:end)'],10);
    rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
        gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
        'LineWidth',6);
end
hold off

%% Initialize realtime environment

% figure(3)
subplot(1,2,2)
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Real Time'),xlabel('x'),ylabel('y')
hold on
plot(xGoal,yGoal,'dm','MarkerSize',12)
plot(xObs,yObs,'xr','MarkerSize',15)
pos = zeros(M,1);
for m = 1:M
    pos(m) = plot(x0{m}(1),x0{m}(2),'ko','MarkerSize',12);
end


pause
%% Run  plan

pointCounter = 2*ones(M,1); % Starts in current cell.
kFin = zeros(M,1);
newStart = start;
mStopped = zeros(M,1); % used to determine priority

if MakeMovie == 1
movieObj = VideoWriter(MovieName);
open(movieObj);

frame = getframe(gcf);
writeVideo(movieObj,frame);
end

for k = 1:N-1
    
    clc
    fprintf('%.0f',k)

    for m = 1:M
        
        mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m));
        
        % stop if achieved goals
        if isempty(goalsLeft{m})
            continue
        end
        
%         cOccupied = [];
%         % collison path detection
%         for m2 = 1:M
%             if m2 ~= m
%                 cOccupied = [cOccupied mTrack{m2}]; % occupied cells
%             end
%         end
        
%         for m2 = 1:M
%             if m2~=m
%                 if ismember(cPath{m}(pointCounter(m)),mTrack{m2}) && mStopped(m2) ~= 0
%                     mStopped(m) = -1;
%                 end
%                 if mStopped(m) ~= 0
%                     break
%                 end
%             end
%         end
        
        for m2 = 1:M
            if m2 ~= m && ismember(cPath{m}(pointCounter(m)),mTrack{m2}) && mStopped(m2)== 0
                mStopped(m) = -1;
                break
%             elseif mStopped(m) ~= 0;
%                 mStopped(m) = 0;
            elseif m2 ~= m && ~ismember(cPath{m}(pointCounter(m)),mTrack{m2}) && mStopped(m) ~= 0
                mStopped(m) = 0;
            end
        end
        
        if mStopped(m) ~= 0
            x{m}(:,k+1) = x{m}(:,k);
            continue
        end
                    
                    
%                     
%         
%         if ismember(cPath{m}(pointCounter(m)),cOccupied) && 
%             mStopped(m) = 1; % 
%             continue
%         end
%         mStopped(m) = 0; % 
%         
        
        kFin(m) = k; % records finishing timesteps
        
        % update position
        u{m}(:,k) = -K*(x{m}(:,k)-[xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]);
        u{m}(:,k) = u{m}(:,k)/norm(u{m}(:,k)+eps);
        x{m}(:,k+1) = A*x{m}(:,k) + B*u{m}(:,k);
        
        % update agent position on graph
        delete(pos(m))
        pos(m) = plot(x{m}(1,k+1),x{m}(2,k+1),'ko','MarkerSize',12);
        pause(.0001)
        
        % if next cell has been reached
        if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
                abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
            % if reached cell is a goal, update goal parameters
            if ismember(cPath{m}(pointCounter(m)),goalsLeft{m})
                goalsReached{m} = [goalsReached{m}, cPath{m}(pointCounter(m))];
                goalsReachedAll = [goalsReachedAll, cPath{m}(pointCounter(m))];
                goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
            end
            % eitherway, update pointCounter
            pointCounter(m) = pointCounter(m) + 1;
        end
        
        % if next cell contains an obstacle (obstacle is sensed)
        if ismember(cPath{m}(pointCounter(m)),obstacles)
            newStart(m) = cPath{m}(pointCounter(m)-1);
            sensedObstacles = [sensedObstacles, cPath{m}(pointCounter(m))];
            
            % rewrite specification
            ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m}(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
            for i = 2:length(goalsLeft{m})
                ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goalsLeft{m}(i)),')'];
            end
            ltlspec{m} = [ltlspec{m}, ' ))'];
            
            % recalculate path
            cPath{m} = [];
            expansion = -1;
            while isempty(cPath{m})
                expansion = expansion + 1;
                gridChunk(m,:) = getChunk(newStart(m), goalsLeft{m}, gridWidth, gridLength, expansion);
                makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), newStart(m), sensedObstacles, ltlspec{m});
                [~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
                [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
            end
            
            pointCounter(m) = 2;
            
            % update sensed obstacle positions
            [xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
            
            % plot new path and obstacle
            subplot(1,2,1)
            hold off
            plot(xGrid,yGrid,'k')
            axis equal
            axis([0 gridWidth+1 0 gridLength+1])
            title('Planning'),xlabel('x'),ylabel('y')
            hold on
            plot(xGoal,yGoal,'dm','MarkerSize',12)
            plot(xSObs,ySObs,'xr','MarkerSize',15)
            for m2 = 1:M % nested m (agent) loop
                plot(xPath{m2}(1),yPath{m2}(1),'ko',xPath{m2}(end),yPath{m2}(end),'k^',...
                    'MarkerSize',12)
                arrow([xPath{m2}(1:end-1)',yPath{m2}(1:end-1)'],...
                    [xPath{m2}(2:end)',yPath{m2}(2:end)'],10);
                rectangle('Position',[gridChunk(m2,1)-.5, gridChunk(m2,3)-.5, ...
                    gridChunk(m2,2)-gridChunk(m2,1)+1, gridChunk(m2,4)-gridChunk(m2,3)+1],...
                    'LineWidth',6);
            end
            hold off
            subplot(1,2,2)
            hold on
        end
    end
        
    if length(goalsReachedAll) == length(goalsAll)
        break
    end
    
    pause(dt-toc)
    
    if MakeMovie == 1
    frame = getframe(gcf);
    writeVideo(movieObj,frame);
    end
    
end

clc
disp(kFin*dt)
toc(timeStart)

%% Plot Final Paths

% figure(4)
subplot(1,2,2)
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Final Path'),xlabel('x'),ylabel('y')
hold on
plot(xGoal,yGoal,'dm','MarkerSize',12)
plot(xObs,yObs,'xr','MarkerSize',15)
for m = 1:M
    plot(xStart(m),yStart(m),'ks')%,xPath{m}(pointCounter(m)-1),yPath{m}(pointCounter(m)-1),'ko',...
%         'MarkerSize',12)
    plot(x{m}(1,1:kFin(m)),x{m}(2,1:kFin(m)),'k-','LineWidth',3)
end
hold off

%%
if MakeMovie == 1
frame = getframe(gcf);
writeVideo(movieObj,frame);
close(movieObj)
end





















