% HumanControlScript
% Run human control of agent

%% Dynamics

kFin(m) = k; % records finishing timesteps (i.e. skipped once agent is finished)

u{m}(:,k) = -K*(x{m}(:,k)-[xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]);
% Calculate input via dlqr
u{m}(:,k) = u{m}(:,k)/norm(u{m}(:,k)+eps); % normalize for constant speed
x{m}(:,k+1) = A*x{m}(:,k) + B*u{m}(:,k); % update position

obsTol = .5; % tolerance before risking contact
senseTol = 1; % tolerance before being sensed

%% Real Time Plot Update

delete(pos(m,:)) % Delete current agent image
% pos(m,1) = plot(x{m}(1,k+1),x{m}(2,k+1),'ko','MarkerSize',12); % update agent image
if m == 1
    pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'b-');
    pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'b--');
    pause(.0001) % ensure plot updates
elseif m == 2
    pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'g-');
    pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'g--');
    pause(.0001) % ensure plot updates
elseif m == 3
    pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'r-');
    pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'r--');
    pause(.0001) % ensure plot updates
end
%% Path Updating

% if next cell has been reached
if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
        abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
    
    % if marked for (human control) release
    if hLatch ~= 1
        hOcc = -1; % release operator; mark as cooldown; return to autonomy
        hReacStart = k; % begin human cooldown counter
        pointCounter(m) = pointCounter(m) + 1; % increment due to how newStart is set up
        
        % Reassign goals?
        if autoPlay == 1
            
        else
            fprintf('\nGoalsReachedAll\n')
            disp(goalsReachedAll);
            fprintf('GoalsLeft\n')
            disp(goalsAll(~ismember(goalsAll,goalsReachedAll)))
            
            if autoplay == 0 && input('\n Reassign Goals? [0 1]: ')
                goals = cell(3,1);
                fprintf('\nGoals for:\n')
                for m2 = 1:M
                    fprintf('\nAgent %.0f',m2)
                    goals{m2} = input(': ');
                    goalsLeft(m2) = goals(m2);
                    if isempty(goals{m2})
                        mTrack{m2} = [cPath{m2}(pointCounter(m2)-1),cPath{m2}(pointCounter(m2)-1)];
                        mStopped(m2) = -3;
                        gridChunk(m2,:) = [-1 -1 1 1];
                    else
                        ObstacleReplanScript2
                    end
                end
            end
        end
%         ObstacleReplanScript; % Replan
        continue
    end
    
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
            hOcc = -1; % release operator; mark as cooldown; return to autonomy
            hLatch = 0;
            hReacStart = k+1/dt; % begin human cooldown counter, (extra added for movie purpose)
            continue
        end
    end
    % eitherway, update pointCounter
    pointCounter(m) = pointCounter(m) + 1;
end

%% If Obstacle is Sensed
% Calculate the distance from each obstacle
obsDis = sqrt((xObs - x{m}(1,k)).^2 + (yObs - x{m}(2,k)).^2);

% if an obstacles is sensed
if sum(obsDis < senseTol) > 0
    % fish out sensed obstacles
    obsCell = obstacles(obsDis<obsTol);
    % add new obstacle(s) to sensedObstacles
    sensedObstacles{m} = [sensedObstacles{m}, obsCell(~ismember(obsCell,sensedObstacles{m}))];
    rewardCount(m) = rewardCount(m) + sum(ismember(obsCell,obstacles(~ismember(obstacles,sensedObstacles{m}))));
    % give 1 reward pt for each new obstacle sensed
end

% if any obstacle is ever "too close for comfort"
if sum(obsDis < obsTol) > 0 && hLatch == 1;
    % mark for return to previous cell
    pointCounter(m) = pointCounter(m)-1;
    % release human plan for return to autonomy
    hLatch = 0;
end


























