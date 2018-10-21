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
    % pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,2) = text(x{m}(1,k+1)+.15,x{m}(2,k+1)+.25,num2str(m));
    set(pos(m,2),'fontsize',16)
    pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'b--');
    pause(.0001) % ensure plot updates
elseif m == 2
    pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'g-');
    % pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,2) = text(x{m}(1,k+1)+.15,x{m}(2,k+1)+.25,num2str(m));
    set(pos(m,2),'fontsize',16)
    pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'g--');
    pause(.0001) % ensure plot updates
elseif m == 3
    pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'r-');
    % pos(m,2) = text(x{m}(1,k+1)-.055,x{m}(2,k+1),num2str(m)); % update agent label
    pos(m,2) = text(x{m}(1,k+1)+.15,x{m}(2,k+1)+.25,num2str(m));
    set(pos(m,2),'fontsize',16)
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

            
        
        fprintf('\nGoalsReachedAll\n')
        disp(goalsReachedAll); % show reached goals
        fprintf('GoalsLeft\n')
        goalsLeftAll = goalsAll(~ismember(goalsAll,goalsReachedAll));
        disp(goalsLeftAll) % show remaining goals
        fprintf('Trust Levels\n')
        disp(mTrust(:,k)')
        goalsAlloc = [1;1;1];% shortcut here for quick debugging
        mTrustMax = find(mTrust(:,k)==max(mTrust(:,k))); % get max trust to reallocate to
        mTrustOrdered = sort(mTrust(:,k),'descend'); % sort trust values for allocation order
        
        
        if autoPlay == 1 || input('\nReassign Goals? [0 1]: ')
            goals = cell(3,1); % if yes, reinitialize
            goalsAlloc(m) = goalsAlloc(m) - 1; % take from lost trust agent
            goalsAlloc(mTrustMax) = goalsAlloc(mTrustMax) + 1; % and give to most trusted
            
            for m1 = 1:M % for each agent
                m2 = find(mTrustOrdered(m1)==mTrust(:,k)); % sorted by trust
                for i = 1:goalsAlloc(m2) % assign this number of goals
                    % Determine pos of remaining goals
                    [xGoalsLeft,yGoalsLeft] = cellPath2Grid(goalsLeftAll,gridWidth,gridLength);
                    % distance of current agent to each goal
                    mGDist = sqrt((x{m2}(1,k) - xGoalsLeft).^2 + (x{m2}(2,k) - yGoalsLeft).^2);
                    % Add the closest goal to goals
                    goals{m2} = [goals{m2} goalsLeftAll(mGDist==min(mGDist))];
                    % Remove from list and repeat if multiple goals
                    goalsLeftAll = goalsLeftAll(~ismember(goalsLeftAll,goals{m2}));
                end
                %                     m2
                goalsLeft{m2} = goals{m2};
                %                     disp(goalsLeft{m2})
                %                     pause
                if isempty(goals{m2})
                    mTrack{m2} = [cPath{m2}(pointCounter(m2)-1),cPath{m2}(pointCounter(m2)-1)];
                    mStopped(m2) = -3;
                    gridChunk(m2,:) = [-1 -1 1 1];
                    cPath{m2} = cPath{m2}(pointCounter(m2)-1);
                    [xPath{m2},yPath{m2}] = cellPath2Grid(cPath{m2},gridWidth,gridLength);
                else
                    ObstacleReplanScript2
                end
            end
        end
        
        %continue
    end
    
    
    

            
            
%             if input('\n Reassign Goals? [0 1]: ')
%                 goals = cell(3,1);
%                 fprintf('\nGoals for:\n')
%                 for m2 = 1:M
%                     fprintf('\nAgent %.0f',m2)
%                     goals{m2} = input(': ');
%                     goalsLeft(m2) = goals(m2);
%                     if isempty(goals{m2})
%                         mTrack{m2} = [cPath{m2}(pointCounter(m2)-1),cPath{m2}(pointCounter(m2)-1)];
%                         mStopped(m2) = -3;
%                         gridChunk(m2,:) = [-1 -1 1 1];
%                     else
%                         ObstacleReplanScript2
%                     end
%                 end
%             end
%         end
% %         ObstacleReplanScript; % Replan
%         continue
%     end
%     
    
    % if reached cell is a goal, update goal parameters
    if ismember(cPath{m}(pointCounter(m)),goalsLeft{m})
        rewardCount(m) = rewardCount(m) + 2; % reward for achieving goal
        goalsReached{m} = [goalsReached{m}, cPath{m}(pointCounter(m))];
        goalsReachedAll = [goalsReachedAll, cPath{m}(pointCounter(m))];
        goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
        fillGoal(cPath{m}(pointCounter(m)),.4,gridWidth,gridLength,'m');
        % if this is the last goal
        if isempty(goalsLeft{m})
            % update the mTrack to show it's not moving
            mTrack{m} = [cPath{m}(pointCounter(m)),cPath{m}(pointCounter(m))];
            hOcc = -1; % release operator; mark as cooldown; return to autonomy
            hLatch = 0;
            hReacStart = k+1/dt; % begin human cooldown counter, (extra added for movie purpose)
            %continue
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
    obsCell = obstacles(obsDis<senseTol);
    % add new obstacle(s) to sensedObstacles
%     sensedObstacles{m} = [sensedObstacles{m}, obsCell(~ismember(obsCell,sensedObstacles{m}))];
    [xSObs{m},ySObs{m}] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength); % update sObs positions
    rewardCount(m) = rewardCount(m) + sum(ismember(obsCell,obstacles(~ismember(obstacles,sensedObstacles{m}))));
    % give 1 reward pt for each new obstacle sensed
    if sum(ismember(obsCell,obstacles(~ismember(obstacles,sensedObstacles{m})))) > 0
        sensedObstacles{m} = [sensedObstacles{m}, obsCell(~ismember(obsCell,sensedObstacles{m}))];
        [xSObs{m},ySObs{m}] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength);
        delete(hObs{m});
        hObs{m} = obsPlot(xSObs{m},ySObs{m},.3,'k',3);
    end
end

% if any obstacle is ever "too close for comfort"
if sum(obsDis < obsTol) > 0 && hLatch == 1;
    rewardCount(m) = rewardCount(m) - 3;
    % mark for return to previous cell
    pointCounter(m) = pointCounter(m)-1;
    % release human plan for return to autonomy
    hLatch = 0;
end


























