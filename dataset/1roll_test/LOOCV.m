function [F_MicroEns, F_MicroOne] = LOOCV(data, labels, noNeighbors)

observations = unique(labels(:, 1));
predictEns = [];
predictOne = [];

split = cvpartition(observations, 'leaveout');

fprintf('\nPerforming LOOCV...'); tic;
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

    % train classifiers
    KNNs = ensembleTrainKNN(trainData, trainLbls, noNeighbors, clusters);
    % add predictions [trueClass, sumClass]
    predictEns = [predictEns; ensemblePredictKNN(KNNs, testData, testLbls, classes)];
    
    % train one classifier
    oneKNN = ClassificationKNN.fit(trainData, trainLbls(:, 3), ...
        'NumNeighbors', noNeighbors, ...
        'Distance', 'euclidean', ...
        'DistanceWeight', 'squaredinverse');
    predictOne = [predictOne ;ensembleOnePredictKNN(oneKNN, testData, testLbls, classes)];
end
fprintf('done\n'); toc;

[confEns] = confusionmat(predictEns(:, 1), predictEns(:, 2));
[~, precEns, recEns] = statsPerClass(confEns);
%F_MacroEns = mean(f1Ens);
F_MicroEns = 2 * (precEns .* recEns) / (precEns + recEns);

[confOne] = confusionmat(predictOne(:, 1), predictOne(:, 2));
[~, precOne, recOne] = statsPerClass(confOne);

F_MicroOne = 2 * (precOne .* recOne) / (precOne + recOne);

fprintf('Leave-one-out ENS Sum F1_mu = %6.4f\n', F_MicroEns);
fprintf('Leave-one-out ONE Sum F1_mu = %6.4f\n', F_MicroOne);
end