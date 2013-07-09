function predictions = ensembleOnePredictKNN(KNN, data, labels, classes )

    observations = unique(labels(:, 1));
    noObservations = numel(observations);
    noClasses = numel(classes);
    predictions = zeros(noObservations, 2);

    % for each observation (complete face) in query data
    for i = 1:noObservations 
        % select patch data of this frame
        idx = strcmp(labels(:, 1), observations(i));
        frameData = data(idx, :);
        frameLbls = labels(idx, :);
        trueClass = frameLbls(1, 3);
        
        % this should be evaluated after each frame is captured from sensor
        patchLabels = cell2mat(labels(idx, 4));
        noPatches = numel(patchLabels);
        
        % sum rule over posterior prob.
        scores = zeros(1, noClasses);
        % majority voting
        % countVotes = zeros(1, noClasses);
        
        % take each patch and select its classifier
        for j = 1:noPatches
            [~, score] = predict(KNN, frameData(j, :));
            scores = scores + score;
            
            % add 1 vote for predicted class
            % countVotes = countVotes + strcmpi(classes, vote)';
        end
        [~, indS] = max(scores);
        % [~, indV] = max(countVotes);
        predictions(i,:) = [find(ismember(classes, trueClass)), indS];
        
%         if indV ~= indS
%             fprintf('\nTrue: %s \t %i Sum: %s \t %i Vote: %s', ...
%             trueClass{1}, strcmp(trueClass{1}, classes{indS}), ...
%             classes{indS}, strcmp(trueClass{1}, classes{indV}), ...
%             classes{indV});
%         end
    end
end