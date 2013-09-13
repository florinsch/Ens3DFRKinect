function [predictions, scorez] = ensemblePredictKNN( KNNs, data, labels, classes, centroids )

    observations = unique(labels(:, 1));
    noObservations = numel(observations);
    noClasses = numel(classes);
    predictions = zeros(noObservations, 2);

    % for each observation (complete face) in query data
    parfor i = 1:noObservations 
        % select patch data of this frame
        idx = strcmp(labels(:, 1), observations(i));
        frameData = data(idx, :);
        frameLbls = labels(idx, :);
        trueClass = frameLbls(1, 3);
        trueClassIdx = find(ismember(classes, trueClass));
        
        % this should be evaluated after each frame is captured from sensor
        patchLabels = cell2mat(labels(idx, 4));
        % make patches unique instead
%         patchLabels = getUniqueClusterLabels(frameData, centroids);
        
        noPatches = numel(patchLabels);
        % sum rule over posterior probabilities
        scores = zeros(1, noClasses);
        
        % majority voting (Worse than sum for M > 4)
        % countVotes = zeros(1, noClasses);
        
        % take each patch and select its classifier
        for j = 1:noPatches
            % select classifier according to region label
            idxKNN = patchLabels(j);
            [~, score] = predict(KNNs{idxKNN}, frameData(j, :));
            
            % some classifiers know less classes
            insertIdx = ismember(classes, KNNs{idxKNN}.ClassNames);
            result = zeros(1, noClasses);
            result(insertIdx) = score;
            scores = scores + result; % sum rule
            
            % keep scores to train perceptrion for weights
            results{i}(j,:) = [result, trueClassIdx, idxKNN];
            
            % add 1 vote for predicted class
            % countVotes = countVotes + strcmpi(classes, vote)';
        end
        [~, indS] = max(scores);        
        % [~, indV] = max(countVotes);
        predictions(i,:) = [trueClassIdx, indS];
        
%         if indV ~= indS
%             fprintf('\nTrue: %s \t %i Sum: %s \t %i Vote: %s', ...
%             trueClass{1}, strcmp(trueClass{1}, classes{indS}), ...
%             classes{indS}, strcmp(trueClass{1}, classes{indV}), ...
%             classes{indV});
%         end
    end
    scorez = cell2mat(results');
end