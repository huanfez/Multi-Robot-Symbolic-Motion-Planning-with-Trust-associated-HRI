% Redesigned with hopefully less headaches...

clear, clc

MakeMovie = 0;
MovieName = 'D:\ExAbsCollisionTest_RT_v2_Slow.avi';
frameRate =  20;%1/.02;%
autoPlay = 0;

numOFigs = 2;

hReacTime = 2;
humanPlanXY = cell(2,1);
humanPlanXY{1} = [3, 10; 4, 9; 5, 10; 10, 10]';
humanPlanXY{1} = [humanPlanXY{1}; 93, 84, 95, 100]; % applied to Agent 2
humanPlanXY{2} = [5,6; 4,5; 4,4; 5,3]';
humanPlanXY{2} = [humanPlanXY{2}; 55, 44, 34, 25]; % applied to Agent 1
humanPlanXY{3} = [10 10; 9 10; 90 100]; % applied to Agent 2

%% Movie elements
timeStart = tic;
autoInput = 1;
%% Environment

gridWidth = 10;
gridLength = 10;
start = [11;91;20]; % per agent
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 94, 80, 70, 69, 48, 38, 95];
goals = [{[25, 54]}; {[84, 100]}; {[17, 87]}];% agent per row

M = 3; % number of agents used

goalsAll = []; % combine all goals into one to determine with to finish
for m = 1:M;
    goalsAll = [goalsAll goals{m}];
end

goalsReached = cell(M,1);
goalsReachedAll = [];
goalsLeft = goals;
goalTol = .125; % "radius" of accepting box in cell

sensedObstacles = cell(M,1); % sensed obstacles begin as empty and seperate

% Conversion from cell to cartesion
xSObs = cell(M,1);
ySObs = cell(M,1);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goalsAll, gridWidth, gridLength);
[xStart,yStart] = cellPath2Grid(start', gridWidth, gridLength);

%% Agents
% Section creates dynamics and control policy (LQR Based)

mR = .125; % agent radius
sR = 1; % sensor radius

% Time
dt = .02; % sec
T = 1000; % sec
N = T/dt; % timesteps (Horizon)

% Dynamics
A = eye(2);
B = dt*eye(2);
Q = eye(2);
R = eye(2);
x = cell(M,1); % cell to store array for each agent
u = cell(M,1); % cell to store inputs for each agent

for m = 1:M % assign x0 to each agent
    [x0,y0] = cellPath2Grid(start(m),gridWidth,gridLength); % change cell to grid coords
    x{m} = zeros(2,N); % initialize state vectors
    x{m}(:,1) = [x0;y0];
    u{m} = zeros(2,N-1); % initialize input vectors
end

K = dlqr(A,B,Q,R); % u = -Kx, x(k+1) = Ax(k) + Bu(k) = (A-BK)x(k)

%% Model Checker Setup

ltlspec = cell(M,1); % Initialize ltlspecs for SMV model
for m = 1:M
    ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goals{m}(1)),')'];
    for i = 2:length(goals{m}) % i here reperesenting goal.  If only one goal, loop is skipped
        ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goals{m}(i)),')'];
    end
    ltlspec{m} = [ltlspec{m}, ' ))'];
end

% file locations
fileName = 'MultiTest.smv';
filePath = cd;
pathNuSMV = 'C:\Program Files\NuSMV\2.5.4\bin'; % Path to NuSMV bin folder

%% Plot Planning Environment
% This is the initialized plot for the planning figure

% Create gridlines.  Offset by -.5 to create boxes around points
xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;

figure(1)
if numOFigs == 1
    subplot(1,2,1) % Use if want one figure, comment out otherwise
end

hold off % To ensure figure is refreshed
plot(xGrid,yGrid,'k') % plot grid lines
axis equal % to make proportions proper
axis([0 gridWidth+1 0 gridLength + 1]);
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12) % Plot goals as magenta diamonds
plot(xStart,yStart,'ok','MarkerSize',12) % plot agents as black circles
goalPlot(xGoal,yGoal,.4,'m');
title('Pathing'),xlabel('x'),ylabel('y')
hold off

pause(.0001) % to ensure update of plot

%% Initial Planning

xPath = cell(M,1); % path in cartesian x
yPath = cell(M,1); % path in cartesian y
cPath = cell(M,1); % path in cell
gridChunk = zeros(M,4); % smallest rectangle required to produce plan
mTrack = cell(M,1); % for collision comm, stores current and next position in plan

for m = 1:M % for each agent, m
    expansion = -1; % used to expand gridChunk if required
    while isempty(cPath{m}) % while no feasible plan exists for current gridChunk
        expansion = expansion + 1; % expand the grid
        gridChunk(m,:) = getChunk(start(m), goals{m}, gridWidth, gridLength, expansion);
        hold on % still on Figure 1, planning environment
        rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
            gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
            'LineWidth',6)
        hold off % plots gridChunk for view of progression in planning
        pause(.1) % ensure plot update, slow enough to view growth
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), start(m), sensedObstacles{m}, ltlspec{m});
        % update SMV model according to new chunk
        [~, output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
        % run model checking
        [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
        % back out path from output of model checker
    end
    mTrack{m} = cPath{m}(1:2); % update collision comm variable
end

%% Plot Initial Plan

% Still on figure 1
hold off % reset figure
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
% title('Planning')
xlabel('x'),ylabel('y')
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');
for m = 1:M
    cEnd = find(ismember(cPath{m},goalsLeft{m}),1,'last');
    plot(xPath{m}(1),yPath{m}(1),'ko',xPath{m}(cEnd),yPath{m}(cEnd),'k^',...
        'MarkerSize',12) % Plot start and end points of path
    arrow([xPath{m}(1:cEnd-1)',yPath{m}(1:cEnd-1)'],...
        [xPath{m}(2:cEnd)',yPath{m}(2:cEnd)'],10); % plot path as arrows
    rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
        gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
        'LineWidth',6); % plot final gridChunk
end
hold off

%% Initialize Realtime Environment

if numOFigs == 2
    figure(2); % if wanted on one figure
else
    subplot(1,2,2) % if wanted on one figure
end

hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
% title('Real Time')
xlabel('x'),ylabel('y')
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');
% plot(xObs,yObs,'xr','MarkerSize',15) % Plot obstacles (not initially available to the planner)
obsPlot(xObs,yObs,.3,'r',3);
pos = zeros(M,3); % stores agent plot marker.  used for delete function to update marker
for m = 1:M
%     pos(m,1) = plot(x{m}(1,1),x{m}(2,1),'ko','MarkerSize',12);
    pos(m,1) = circle(x{m}(1,1),x{m}(2,1),mR,'k-');
    pos(m,2) = text(x{m}(1,1)-.075,x{m}(2,1),num2str(m));
    pos(m,3) = circle(x{m}(1,1),x{m}(2,1),sR,'b--');
end

clc
fprintf('Press any key to continue\n')
pause % allow adjustment of figures before running

%% Plan Variable Initialization

pointCounter = 2*ones(M,1); % used for tracking progression in x/y/cPath
kFin = zeros(M,1); % used for tracking when agent has completed all goals
newStart = start; % used to update plan
mStopped = zeros(M,1); % used to determine collision priority
rewardCount = zeros(M,1)-2; % Reward value for tracking (work in progress)
hOcc = 0; % Human occupied tracker (1 when occupied)

%% Run Plan

if MakeMovie == 1
    if numOFigs ~= 1
        error('change numOFigs to 1')
    end
    movieObj = VideoWriter(MovieName);
    movieObj.FrameRate = frameRate;
    open(movieObj)
    frame = getframe(gcf);
    writeVideo(movieObj,frame);
end

for k = 1:N-1
    kTime = tic; % used to slow down to real time if too fast
    clc
    fprintf('%.0f',k) % print current step
    
% Collision == 416, HumanReplan1 == 97
%     if k == 97
%         [1];
%     end
    
    for m = 1:M % for each agent
        
        if isempty(goalsLeft{m}) % if all goals are achieved
            continue % agent stops
        end
        % otherwise...
        
        %% Should human take over?
        
        % Allow new agent to be chosen after cooldown
        if hOcc == -1 && dt*(k-hReacStart) >= hReacTime
            hOcc = 0;
        end
        
        % Trust calculation
        if hOcc == 0
            obsDisAll = zeros(M,length(obstacles));
            obsComplex = zeros(M,1);
            % Determine distance of each agent to all obstacles
            for m2 = 1:M
                obsDisAll(m2,:) = sqrt((xObs - x{m2}(1,k)).^2 + (yObs - x{m2}(2,k)).^2);
                obsComplex(m2,:) = sum(obsDisAll(m2,:)<1.1);
            end
            mTrust = rewardCount + (1-.5.^(obsComplex+1));
        end
            
        
        
%         if hOcc == 0 && rewardCount(m) > 0 && rewardCount(m) == max(rewardCount)
        if hOcc == 0 && mTrust(m) > 0 && mTrust(m) == max(mTrust)
            hOcc = m; % mark human as occupied
            % Request new path from operator
            fprintf('\nPlan for agent %.0f',m);
            if autoPlay == 1
                hPlan = humanPlanXY{autoInput};
                autoInput = autoInput + 1;
            else
                hPlan = input(': '); % input as [xPath;yPath;cPath]
            end
            % Update Plan to Human Input
            xPath{m} = hPlan(1,:);
            yPath{m} = hPlan(2,:);
            cPath{m} = hPlan(3,:);
            gridChunk(m,:) = [-1 -1 1 1];
            pointCounter(m) = 2;
            hLatch = 1; % mark human plan as being implemented
            hReacStart = k;
            ReplottingScript
        end
        
        if m == hOcc
            if dt*(k-hReacStart) >= hReacTime % once human has had enough time to plan
                HumanControlScript_v2 % run human plan version of code
            else % if planning time not complete
                x{m}(:,k+1) = x{m}(:,k);
            end % eitherway, 
            continue % Skip everything else of this agent (autonomy aspects)
        end
        
        
        
        %% Collision detection
        
        mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m)); % is this necessary to do here?
        % update collision tracker
        for m2 = 1:M % for each other agent
            if m2 ~= m && ismember(cPath{m}(pointCounter(m)),mTrack{m2})
            % if m is going to conflict with another agent, m2
                mStopped(m) = -1; % flag agent m to pause
                % if mTrack{m}(2) == mTrack{m2}(1) && mTrack{m2}(2) == mTrack{m}(1) % I like how this one works better 
                if (mTrack{m}(2) - mTrack{m}(1)) == -(mTrack{m2}(2) - mTrack{m2}(1))
                % Unless m and m2 are moving at each other
                    mStopped(m) = -2; % flag agent m to replan
                end
                break % collision detected.  continue with plan
            elseif m2 ~= m && ~ismember(cPath{m}(pointCounter(m)),mTrack{m2})
            % otherwise, if no collision detected (i.e. no longer detected)
                mStopped(m) = 0; % flag to resume normal operation
            end
        end
        
        if mStopped(m) == -1 && mStopped(m2) == -1
        % if both m and m2 are flagged to pause
            if mTrack{m}(2) ~= mTrack{m2}(1)
            % if agent m will not be moving into m2's current cell
                mStopped(m) = 0; % flag to continue
            end
        end
        
        %% Replanning For Collision Avoidance
        
        if mStopped(m) == -1
        % if flagged to pause
            x{m}(:,k+1) = x{m}(:,k); % update position
            continue % skip rest of code
        end
        
        if mStopped(m) == -2
        % if flagged to replan
            CollisionReplanScript_v2 % Replan (seperated for sanity)
        end
        
        %% Dynamics
        
        kFin(m) = k; % records finishing timesteps (i.e. skipped once agent is finished)
        
        u{m}(:,k) = -K*(x{m}(:,k)-[xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]);
        % Calculate input via dlqr
        u{m}(:,k) = u{m}(:,k)/norm(u{m}(:,k)+eps); % normalize for constant speed
        x{m}(:,k+1) = A*x{m}(:,k) + B*u{m}(:,k); % update position
        
        %% Real Time Plot Update
        delete(pos(m,:)) % Delete current agent image
%         pos(m,1) = plot(x{m}(1,k+1),x{m}(2,k+1),'ko','MarkerSize',12); % update agent image
        pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'k-');
        pos(m,2) = text(x{m}(1,k+1)-.075,x{m}(2,k+1),num2str(m)); % update agent label
        pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'b--');
        pause(.0001) % ensure plot updates
        
        %% Path Updating
        
        % if next cell has been reached
        if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
                abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
            % if reached cell is a goal, update goal parameters
            if ismember(cPath{m}(pointCounter(m)),goalsLeft{m})
                rewardCount(m) = rewardCount(m) + 2; % reward for achieving goal
                goalsReached{m} = [goalsReached{m}, cPath{m}(pointCounter(m))];
                goalsReachedAll = [goalsReachedAll, cPath{m}(pointCounter(m))];
                goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
                % if this is the last goal
                if isempty(goalsLeft{m})
                    % update the mTrack to show it's not moving
                    mTrack{m} = [cPath{m}(pointCounter(m)),cPath{m}(pointCounter(m))];
                end
            end
            % eitherway, check for obstacles around you
            % if any adjacent cells contain an obstacle
            cSurround = [cPath{m}(pointCounter(m))-1, cPath{m}(pointCounter(m))+1, cPath{m}(pointCounter(m))-gridWidth, cPath{m}(pointCounter(m))+gridWidth];
            rewardCount(m) = rewardCount(m) + sum(ismember(cSurround,obstacles(~ismember(obstacles,sensedObstacles{m}))));
            % give 1 reward pt for each new obstacle sensed
            sensedObstacles{m} = [sensedObstacles{m}, cSurround(ismember(cSurround,obstacles))];
            % and then update pointCounter
            pointCounter(m) = pointCounter(m) + 1;
        end
        
        % if next cell contains an obstacle (obstacle is sensed)
        if ismember(cPath{m}(pointCounter(m)),obstacles) && ~isempty(goalsLeft{m})
%             rewardCount(m) = rewardCount(m) - 1;
            ObstacleReplanScript % Replan (seperated for sanity)
        end
    end % end agent loop
        
    if length(goalsReachedAll) == length(goalsAll) % if all goals have been reached
        break % end operation of agents
    end
    
    pause(dt-toc(kTime)) % pause if loop's faster than real time
%     pause(1/30)
    
    if MakeMovie == 1
        frame = getframe(gcf);
        writeVideo(movieObj,frame);
    end
    
end % end horizon loop

clc
disp(kFin*dt) % display how long each agent took to complete goals
toc(timeStart) % display how long to run the simulation

%% Plot Final Paths

if numOFigs == 2
    figure(2)
else
    subplot(1,2,2)
end
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Final Path'),xlabel('x'),ylabel('y')
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');
% plot(xObs,yObs,'xr','MarkerSize',15)
obsPlot(xObs,yObs,.3,'r',3);
for m = 1:M
    plot(xStart(m),yStart(m),'ks')
    plot(x{m}(1,1:kFin(m)),x{m}(2,1:kFin(m)),'k-','LineWidth',3)
end
hold off

%% 
if MakeMovie == 1
    for i = 1:frameRate
        frame = getframe(gcf);
        writeVideo(movieObj,frame);
    end
    close(movieObj)
end