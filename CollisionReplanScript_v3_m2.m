% CollisionReplanScript

%% Replanning
senseTol = 1;
newStart(m2) = cPath{m2}(pointCounter(m2)-1); % update SMV start position

% Rewrite specification
ltlspec{m2} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m2}(1)),')'];
for i = 2:length(goalsLeft{m2})
    ltlspec{m2} = [ltlspec{m2}, ' & F (x.state = ',num2str(goalsLeft{m2}(i)),')'];
end
ltlspec{m2} = [ltlspec{m2}, ' ))'];

% Recalculate path
cPath{m2} = []; % empty cPath
expansion = -1; % reset expansion
% mTrackSense = [];
% for iter = 1:M
%     if norm(x{iter}(:,k)-x{m2}(:,k))<2*senseTol && iter ~= m2
%         mTrackSense = [mTrackSense mTrack{iter}];
%     end
% end
while isempty(cPath{m2}) % while no path is found
    expansion = expansion + 1;
    gridChunk(m2,:) = getChunk(newStart(m2), goalsLeft{m2}, gridWidth, gridLength, expansion);
    makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m2,:), newStart(m2), sensedObstacles{m2}, ltlspec{m2});
    % Add conflicting agent(s) to obstacles list for this instance only
    [~, output] = system(['NuSMV ' filePath '\' fileName]);
    [xPath{m2},yPath{m2},cPath{m2}] = getPath(output, gridWidth, gridLength);
end

pointCounter(m2) = 2; % reset pointCounter

%% Replotting
ReplottingScript

%% Return to main loop