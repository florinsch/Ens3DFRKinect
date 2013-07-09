function [data, labels] = patchLabelsFromGrid(data, labels)
    fprintf('\nGeting labels from Grid topology...'); tic;
    % extract numbers - HACK!!
    gridLbls = cellfun(@(s) s(14:end), labels(:, 2), 'UniformOutput', false);
    gridLbls = str2double(gridLbls); % convert to double
    gridLbls = gridLbls + 1; % increment +1 (starts from 0)

    % get max idx for a patch
    noClusters = max(gridLbls);
    patchClass = 1:noClusters;

    % get number of items per class
    classesHist = histc(gridLbls, 1:noClusters);

    % keep only clusters with enough observations
    idxKeep = classesHist >= mean(classesHist) - std(classesHist);    

    % filter out classes not having enough examples 
    validClasses = patchClass(idxKeep);
    valid = ismember(gridLbls, validClasses);

    % copy values as numbers
    labels(:, 4) = num2cell(gridLbls);

    % remove clusters with 0 as index
    labels = labels(valid, :);
    data = data(valid, :);
    
    fprintf('done\n'); toc;
end