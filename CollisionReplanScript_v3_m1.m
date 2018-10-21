% CollisionReplanScript

%% Replanning
senseTol = 1;
newStart(m1) = cPath{m1}(pointCounter(m1)-1); % update SMV start position

% Rewrite specification
ltlspec{m1} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m1}(1)),')'];
for i = 2:length(goalsLeft{m1})
    ltlspec{m1} = [ltlspec{m1}, ' & F (x.state = ',num2str(goalsLeft{m1}(i)),')'];
end
ltlspec{m1} = [ltlspec{m1}, ' ))'];

% Recalculate path
cPath{m1} = []; % empty cPath
expansion = -1; % reset expansion
% mTrackSense = [];
% for iter = 1:M
%     if norm(x{iter}(:,k)-x{m1}(:,k))<2*senseTol && iter ~= m1
%         mTrackSense = [mTrackSense mTrack{iter}];
%     end
% end
while isempty(cPath{m1}) % while no path is found
    expansion = expansion + 1;
    gridChunk(m1,:) = getChunk(newStart(m1), goalsLeft{m1}, gridWidth, gridLength, expansion);
    makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m1,:), newStart(m1), sensedObstacles{m1}, ltlspec{m1});
    % Add conflicting agent(s) to obstacles list for this instance only
    [~, output] = system(['NuSMV ' filePath '\' fileName]);
    [xPath{m1},yPath{m1},cPath{m1}] = getPath(output, gridWidth, gridLength);
end

pointCounter(m1) = 2; % reset pointCounter

%% Replotting
ReplottingScript

%% Return to main loop