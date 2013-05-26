function KNNs = trainKNNs( data, labels, noNeighbors, clusters )
    noClusters = numel(clusters);
    KNNs{noClusters} = []; % preallocate

    % for each cluster
    for i = 1:noClusters
        % select observations
        idx = cell2mat(labels(:, 4)) == clusters(i);
        KNNdata = data(idx, :);
        KNNlbls = labels(idx, 3);
        % train KNN
        KNNs{clusters(i)} = ClassificationKNN.fit(KNNdata, KNNlbls, ...
        'DistanceWeight', 'squaredinverse', 'NumNeighbors', noNeighbors);
    end
end

