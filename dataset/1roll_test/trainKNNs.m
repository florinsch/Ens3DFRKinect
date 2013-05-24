function KNNs = trainKNNs( data, labels, noNeighbors, noClusters )
    KNNs{noClusters} = []; % preallocate

    % for each cluster
    for i = 1:noClusters
        % select observations
        idx = cell2mat(labels(:, 4)) == i;
        KNNdata = data(idx, :);
        KNNlbls = labels(idx, 3);
        % train KNN
        KNNs{i} = ClassificationKNN.fit(KNNdata, KNNlbls, ...
        'DistanceWeight', 'squaredinverse', 'NumNeighbors', noNeighbors);
    end
end

