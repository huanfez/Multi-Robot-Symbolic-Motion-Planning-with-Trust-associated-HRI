clear, clc
timeStart = tic;
MakeMovie = 0;
MovieName = 'D:\ExAbsCollaborativeTest_SlowT.avi';
%% Environment

gridWidth = 10;
gridLength = 10;
start = [11;91;20]; % agent per row
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 94, 80, 70, 69, 48, 38, 95];
goals = [{[25, 54]}; {[84, 100]}; {[17, 87]}];% agent per row

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
reacTime = 2;

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
sensedObstacles = cell(3,1);
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
        
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), start(m), sensedObstacles{m}, ltlspec{m});
        
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
pos = zeros(M,2);
for m = 1:M
    pos(m,1) = plot(x0{m}(1),x0{m}(2),'ko','MarkerSize',12);
    pos(m,2) = text(x0{m}(1)-.055,x0{m}(2),num2str(m));
end

clc
fprintf('Press any key to continue\n')
pause
%% Run  plan

pointCounter = 2*ones(M,1); % Starts in current cell.
kFin = zeros(M,1);
newStart = start;
mStopped = zeros(M,1); % used to determine priority

% Human mode simulation parameters
hCounter = 2;
humanPlanXY = cell(2,1);
humanPlanXY{2} = [3, 10; 3.5, 9.5; 4, 9; 5, 10; 10, 10]';
humanPlanXY{1} = [5,6; 5,6; 4,5; 4,4; 5,3; 5,3]';
% [4,4; 4.5, 3.5; 5, 3; 6, 4]';

if MakeMovie == 1
    movieObj = VideoWriter(MovieName);
    movieObj.FrameRate = 20;
    open(movieObj);
    
    frame = getframe(gcf);
    writeVideo(movieObj,frame);
end

for k = 1:N-1
    tic
    
    clc
    fprintf('%.0f',k)
    
    for m = 1:M
        
        mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m));
        
        % stop if achieved goals
        if isempty(goalsLeft{m})
            continue
        end
        
        % collision property
        for m2 = 1:M
            if mStopped(m) < -10
                break
            elseif m2 ~= m && ismember(cPath{m}(pointCounter(m)),mTrack{m2})% && mStopped(m2)== 0
                mStopped(m) = -1; % pause condition
                if (mTrack{m}(2) - mTrack{m}(1)) == -(mTrack{m2}(2) - mTrack{m2}(1))
                    %                     if norm([xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]-x{m}(:,k))...
                    %                             < norm([xPath{m2}(pointCounter(m2));yPath{m2}(pointCounter(m2))]-x{m2}(:,k))
                    %                         mStopped(m2) = -2;
                    %                     else
                    mStopped(m) = -2; % replan condition
                    %                     end
                    %                     break
                end
                break
            elseif m2 ~= m && ~ismember(cPath{m}(pointCounter(m)),mTrack{m2})% && mStopped(m) ~= 0
                mStopped(m) = 0;
            end
        end
        
        if mStopped(m) == -1 && mStopped(m2) == -1
            if mTrack{m}(2) ~= mTrack{m2}(1)
                mStopped(m) = 0;
            end
        end
        
        %         if mStopped(m) == -1
        %             if mTrack{m2}(2) == mTrack{m}(1)
        %                 mStopped(m) = 0;
        %                 mStopped(m2) = -1;
        %             end
        %         end
        
        % human control simulation
        if hCounter == 2 && m == 2 && ismember(83,sensedObstacles{m}) && mStopped(m) > -10% trigger by sensing this obstacle
            % engage human mode on robot 2
            mStopped(m) = -12;
        elseif hCounter == 1 && m == 1 && mStopped(m) > -10 && mTrack{m}(1) == 55 && k > 450% && some tag to indicate robot
            % engage human mode on robot 1
            mStopped(m) = -11;
        end
        
        
        
        if mStopped(m) == -1 % pause condition
            x{m}(:,k+1) = x{m}(:,k);
            continue
        elseif mStopped(m) == -2 % replan condition
            
            newStart(m) = cPath{m}(pointCounter(m)-1);
            
            % rewrite specification
            ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m}(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
            for i = 2:length(goalsLeft{m})
                ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goalsLeft{m}(i)),')'];
            end
            ltlspec{m} = [ltlspec{m}, ' ))'];
            
            % for this comm example
            mTrackAllOthers = [];
            for m3 = 1:M
                if m3 ~= m
                    mTrackAllOthers = [mTrackAllOthers mTrack{m3}];
                end
            end
            
            % recalculate path
            cPath{m} = [];
            expansion = -1;
            while isempty(cPath{m})
                expansion = expansion + 1;
                gridChunk(m,:) = getChunk(newStart(m), goalsLeft{m}, gridWidth, gridLength, expansion);
                % Add conflicting agent to obastacles list for this
                % instance only
                makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), newStart(m), [sensedObstacles{m} mTrackAllOthers], ltlspec{m});
                [~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
                [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
            end
            
            pointCounter(m) = 2;
            
            % update sensed obstacle positions
            %             [xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
            
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
            
        elseif mStopped(m) == -12
            mStopped(m) = -22;
            xPath{m} = humanPlanXY{m}(1,:);
            yPath{m} = humanPlanXY{m}(2,:);
            cPath{m} = ones(1,length(xPath{m}));
            reacTimeStart = k;
        elseif mStopped(m) == -11;
            mStopped(m) = -21;
            xPath{m} = humanPlanXY{m}(1,:);
            yPath{m} = humanPlanXY{m}(2,:);
            cPath{m} = ones(1,length(xPath{m}));
            reacTimeStart = k;
        end
        
        if mStopped(m) == -21 || mStopped(m) == -22
            x{m}(:,k+1) = x{m}(:,k);
            if dt*(k-reacTimeStart) > reacTime
                mStopped(m) = mStopped(m) - 10;
            end
            continue
        end
        
        if mStopped(m) == -32 && x{m}(1,k) > 4.5 && x{m}(2,k)  > 9.5
            pointCounter(m) = pointCounter(m) - 1;
            mStopped(m) = -42;
        end
        
        if mStopped(m) == -31 
            if pointCounter(m) == 6
                mStopped(m) = 0;

            
            mStopped(m) = 0;
            hCounter = 0;
            pointCounter(m) = 2;
            goalsReached{m} = [goalsReached{m}, 25];
            goalsReachedAll = [goalsReachedAll, 25];
            goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
            end

        end
        

        
        kFin(m) = k; % records finishing timesteps
        
        % update position
        u{m}(:,k) = -K*(x{m}(:,k)-[xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]);
        u{m}(:,k) = u{m}(:,k)/norm(u{m}(:,k)+eps);
        x{m}(:,k+1) = A*x{m}(:,k) + B*u{m}(:,k);
        
        % update agent position on graph
        delete(pos(m,:))
        pos(m,1) = plot(x{m}(1,k+1),x{m}(2,k+1),'ko','MarkerSize',12);
        pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m));
        pause(.0001)
        
        % if next cell has been reached
        if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
                abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
            % if reached cell is a goal, update goal parameters
            if ismember(cPath{m}(pointCounter(m)),goalsLeft{m}) && (mStopped(m) ~= -42 || mStopped(m) ~= 41)
                goalsReached{m} = [goalsReached{m}, cPath{m}(pointCounter(m))];
                goalsReachedAll = [goalsReachedAll, cPath{m}(pointCounter(m))];
                goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
            end
            % eitherway, update pointCounter
            pointCounter(m) = pointCounter(m) + 1;
            if mStopped(m) == -42;
                mStopped(m) = mStopped(m) - 10;
                cPath{m}(pointCounter(m)-1:pointCounter(m)) = [84, 95];
%             elseif mStopped(m) == -41;
%                 mStopped(m) = mStopped(m) - 10;
%                 cPath{m}(pointCounter(m)-1:pointCounter(m)) = [25, 26];
            end
        end
        
        % if next cell contains an obstacle (obstacle is sensed)
        if (ismember(cPath{m}(pointCounter(m)),obstacles) && ~isempty(goalsLeft{m})) || mStopped(m) == -52% || mStopped(m) == -51
            if mStopped(m) == -52
                mStopped(m) = 0;
                hCounter = 1;
%                 pointCounter(m) = 2;
                goalsReached{m} = [goalsReached{m}, 84];
                goalsReachedAll = [goalsReachedAll, 84];
                goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
%             elseif mStopped(m) == -51
%                 mStopped(m) = 0;
%                 hCounter = 0;
%                 pointCounter(m) = 2;
%                 goalsReached{m} = [goalsReached{m}, 55];
%                 goalsReachedAll = [goalsReachedAll, 55];
%                 goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
            end
            newStart(m) = cPath{m}(pointCounter(m)-1);
            sensedObstacles{m} = [sensedObstacles{m}, cPath{m}(pointCounter(m))];
            
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
                makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), newStart(m), sensedObstacles{m}, ltlspec{m});
                [~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
                [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
            end
            
            pointCounter(m) = 2;
            
            % update sensed obstacle positions
            [xSObs,ySObs] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength);
            
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
    
%     pause(dt-toc)
    %
    %     if k > 1000
    %         pause(.5)
    %     end
    
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





















