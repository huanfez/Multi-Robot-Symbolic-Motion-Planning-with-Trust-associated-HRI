% CollisionReplanScript

%% Replanning
senseTol = 1;
newStart(m) = cPath{m}(pointCounter(m)-1); % update SMV start position

% Rewrite specification
ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m}(1)),')'];
for i = 2:length(goalsLeft{m})
    ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goalsLeft{m}(i)),')'];
end
ltlspec{m} = [ltlspec{m}, ' ))'];

% Recalculate path
cPath{m} = []; % empty cPath
expansion = -1; % reset expansion
% mTrackSense = [];
% for iter = 1:M
%     if norm(x{iter}(:,k)-x{m}(:,k))<2*senseTol && iter ~= m
%         mTrackSense = [mTrackSense mTrack{iter}];
%     end
% end
while isempty(cPath{m}) % while no path is found
    expansion = expansion + 1;
    gridChunk(m,:) = getChunk(newStart(m), goalsLeft{m}, gridWidth, gridLength, expansion);
    makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), newStart(m), sensedObstacles{m}, ltlspec{m});
    % Add conflicting agent(s) to obstacles list for this instance only
        [~, output] = system(['NuSMV ' filePath '\' fileName]);
    [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
end

pointCounter(m) = 2; % reset pointCounter

%% Replotting
ReplottingScript

%% Return to main loop