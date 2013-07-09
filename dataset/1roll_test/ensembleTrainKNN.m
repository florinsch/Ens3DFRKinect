function KNNs = ensembleTrainKNN( data, labels, noNeighbors, clusters )
    noClusters = numel(clusters);
    KNNs{noClusters} = []; % preallocate
    
    % for each cluster
    for i = 1:noClusters
        % select observations
        idx = cell2mat(labels(:, 4)) == clusters(i);
        % get data
        KNNdata = data(idx, :);
        KNNlbls = labels(idx, 3);        
        % some classifiers see < total classes due to bad clustering
        %unique(KNNlbls);

        % train KNN
        KNNs{clusters(i)} = ClassificationKNN.fit(KNNdata, KNNlbls, ...
        'NumNeighbors', noNeighbors, ...
        'Distance', 'euclidean', ...
        'DistanceWeight', @(d)(d.^-2)); % squaredinverse
    end
end

