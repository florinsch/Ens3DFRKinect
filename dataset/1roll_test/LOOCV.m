function [F_Macro, F_Micro, conf] = LOOCV(data, labels, noNeighbors, ~)

observations = unique(labels(:, 1));
predictEns = [];
predictOne = [];

split = cvpartition(observations, 'leaveout');

single = nargin > 3;

fprintf('\nPerforming LOOCV K_nn = %i ...', noNeighbors); tic;
parfor i = 1:split.NumTestSets
    % select patches from the testing frame
    testIdx = strcmp(labels(:,1), observations(split.test(i)));
    % use everything else for training
    trainIdx = ~testIdx;
    
    % get the data
    trainData = data(trainIdx, :);
    trainLbls = labels(trainIdx, :);
    testData = data(testIdx, :);
    testLbls = labels(testIdx, :);
    
    % valid classes
    classes = unique(trainLbls(:, 3));
    % valid clusters
    clusters = unique(cell2mat(trainLbls(:, 4)));

    if ~single
        % train classifiers
        KNNs = ensembleTrainKNN(trainData, trainLbls, noNeighbors, clusters);
        % add predictions [trueClass, sumClass]
        predictEns = [predictEns; ensemblePredictKNN(KNNs, testData, testLbls, classes)];
    else    
        % train one classifier
        oneKNN = ClassificationKNN.fit(trainData, trainLbls(:, 3), ...
            'NumNeighbors', noNeighbors, ...
            'Distance', 'euclidean', ...
            'DistanceWeight', 'squaredinverse');
        predictOne = [predictOne; ensembleOnePredictKNN(oneKNN, testData, testLbls, classes)];
    end
end
fprintf('done\n'); toc;

if ~single
    [conf] = confusionmat(predictEns(:, 1), predictEns(:, 2));
    fprintf('Ensemble ');
else
    [conf] = confusionmat(predictOne(:, 1), predictOne(:, 2));
    fprintf('One KNN ');
end

[f1, prec, rec] = statsPerClass(conf);
F_Macro = mean(f1);
F_Micro = 2 * (prec .* rec) / (prec + rec);
fprintf('LOOCV Sum F1_M %6.4f | F1_m %6.4f\n', F_Macro, F_Micro);

end