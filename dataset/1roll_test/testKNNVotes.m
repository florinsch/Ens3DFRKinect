function accuracy = testKNNVotes( KNNs, data, labels, classes )

    frames = unique(labels(:, 1));
    noFrames = numel(frames);
    noClasses = numel(classes);
    results = zeros(noFrames, 1);

    % for each frame in provided data
    for i = 1:noFrames 
        % select patches of this frame
        idx = strcmp(labels(:, 1), frames(i));
        frameData = data(idx, :);
        frameLbls = labels(idx, :);
        trueClass = frameLbls(1, 3);

        % this should be evaluated after each frame is captured from sensor
        patchLabels = cell2mat(labels(idx, 4)); 
        noPatches = numel(patchLabels);

        % take each patch and evaluate prediction
        countVotes = zeros(noClasses, 1);
        for j = 1:noPatches
            % select classifier for estimated cluster and evaluate patch
            vote = predict(KNNs{patchLabels(j)}, frameData(j, :));
            % add 1 vote for predicted class
            for k = 1:noClasses
                countVotes(k) = countVotes(k) + strcmpi(classes{k}, vote);
            end
        end
        [~, ind] = max(countVotes);
        results(i) = strcmpi(classes(ind), trueClass);
    end

    accuracy = sum(results) / noFrames;
    
end